#include "lanes_recommendation.hpp"

#include "routing/lanes/lane_way.hpp"
#include "routing/route.hpp"
#include "routing/turns.hpp"

#include "indexer/ftypes_matcher.hpp"

#include <cmath>

namespace routing::turns::lanes
{
namespace
{
void FixRecommendedReverseLane(LaneWays & ways, LaneWay const recommendedWay)
{
  if (recommendedWay == LaneWay::ReverseLeft)
    ways.Remove(LaneWay::ReverseRight);
  else if (recommendedWay == LaneWay::ReverseRight)
    ways.Remove(LaneWay::ReverseLeft);
}
}  // namespace

bool SetRecommendedLanes(CarDirection const carDirection, LanesInfo & lanesInfo)
{
  // Check if there are elements in lanesInfo that correspond with the turn exactly.
  // If so, fix up all the elements in lanesInfo that correspond with the turn.
  if (impl::SetRecommendedLaneWays(carDirection, lanesInfo))
    return true;
  // If not, check if there are elements in lanesInfo that correspond with the turn
  // approximately. If so, fix up all those elements.
  if (impl::SetRecommendedLaneWaysApproximately(carDirection, lanesInfo))
    return true;
  // If not, check if there is an unrestricted lane that could correspond to the
  // turn. If so, fix up that lane.
  return impl::SetUnrestrictedLaneAsRecommended(carDirection, lanesInfo);
}

void SelectRecommendedLanes(std::vector<RouteSegment> & routeSegments)
{
  for (auto & segment : routeSegments)
  {
    auto & t = segment.GetTurn();
    if (t.IsTurnNone() || t.m_lanes.empty())
      continue;
    // Otherwise, we don't have lane recommendations for the user, so we don't
    // want to send the lane data any further.
    if (!SetRecommendedLanes(t.m_turn, segment.GetTurnLanes()))
      segment.ClearTurnLanes();
  }
  if (GetLaneChangeSettings().m_enabled)
    MinimizeLaneChanges(routeSegments, GetLaneChangeSettings());
}

LaneChangeSettings & GetLaneChangeSettings()
{
  static LaneChangeSettings settings;
  return settings;
}

namespace
{
enum class Side
{
  None,
  Left,
  Right
};

bool IsHighway(RouteSegment const & segment)
{
  auto const & rni = segment.GetRoadNameInfo();
  return !rni.m_isLink && (rni.m_highwayClass == ftypes::HighwayClass::Motorway ||
                           rni.m_highwayClass == ftypes::HighwayClass::Trunk);
}

Side SideOf(CarDirection const t)
{
  if (IsTurnMadeFromLeft(t))
    return Side::Left;
  if (IsTurnMadeFromRight(t))
    return Side::Right;
  return Side::None;
}

/// What the junctions before a given point should prepare for.
struct Target
{
  enum class Kind
  {
    None,
    Side,  // A maneuver to the left/right without lane data.
    Lanes  // A junction with lane data: the lanes to be in, counted from both road edges.
  };

  Kind m_kind = Kind::None;
  Side m_side = Side::None;
  // Range of the target lanes as offsets from the left and from the right road edge.
  size_t m_leftFirst = 0, m_leftLast = 0;
  size_t m_rightFirst = 0, m_rightLast = 0;
  double m_distMeters = 0.0;
};

struct Recommended
{
  size_t m_first = 0, m_last = 0, m_count = 0;
};

Recommended GetRecommended(LanesInfo const & lanes)
{
  Recommended r;
  for (size_t i = 0; i < lanes.size(); ++i)
  {
    if (lanes[i].recommendedWay == LaneWay::None)
      continue;
    if (r.m_count == 0)
      r.m_first = i;
    r.m_last = i;
    ++r.m_count;
  }
  return r;
}

void KeepOnlyLane(LanesInfo & lanes, size_t const keep)
{
  for (size_t i = 0; i < lanes.size(); ++i)
    if (i != keep)
      lanes[i].recommendedWay = LaneWay::None;
}

// Keeps only the leftmost (or rightmost) recommended lane.
void KeepOneLane(LanesInfo & lanes, Side const side)
{
  auto const r = GetRecommended(lanes);
  if (r.m_count < 2 || side == Side::None)
    return;
  KeepOnlyLane(lanes, side == Side::Left ? r.m_first : r.m_last);
}

double DistToRange(size_t const v, size_t const first, size_t const last)
{
  if (v < first)
    return static_cast<double>(first - v);
  if (v > last)
    return static_cast<double>(v - last);
  return 0.0;
}

// Keeps the recommended lane which lines up best with the target lanes of the next junction.
// Lanes are aligned at the road edge the target lanes are closer to: turn lanes are usually
// added or dropped at the edges, so counting from the nearer edge matches lanes best.
void KeepAlignedLane(LanesInfo & lanes, Target const & target, bool const preferRight)
{
  auto const r = GetRecommended(lanes);
  if (r.m_count < 2)
    return;

  size_t const n = lanes.size();
  bool const leftAnchored = target.m_leftFirst < target.m_rightLast;
  bool const rightAnchored = target.m_rightLast < target.m_leftFirst;

  size_t best = n;
  double bestCost = 0.0;
  for (size_t i = r.m_first; i <= r.m_last; ++i)
  {
    if (lanes[i].recommendedWay == LaneWay::None)
      continue;
    size_t const fromLeft = i, fromRight = n - 1 - i;
    // The small second term breaks ties towards the anchored edge.
    double cost;
    if (leftAnchored)
      cost = DistToRange(fromLeft, target.m_leftFirst, target.m_leftLast) + 1e-3 * fromLeft;
    else if (rightAnchored)
      cost = DistToRange(fromRight, target.m_rightLast, target.m_rightFirst) + 1e-3 * fromRight;
    else  // Target is in the middle of the road: keep to the middle, too.
      cost = std::abs(static_cast<double>(fromLeft) - static_cast<double>(fromRight)) +
             1e-3 * static_cast<double>(preferRight ? fromRight : fromLeft);
    if (best == n || cost < bestCost)
    {
      best = i;
      bestCost = cost;
    }
  }
  if (best != n)
    KeepOnlyLane(lanes, best);
}

Target MakeLanesTarget(LanesInfo const & lanes, double const distMeters)
{
  Target target;
  auto const r = GetRecommended(lanes);
  // All lanes recommended: nothing to prepare for.
  if (r.m_count == 0 || r.m_count == lanes.size())
    return target;
  size_t const n = lanes.size();
  target.m_kind = Target::Kind::Lanes;
  target.m_leftFirst = r.m_first;
  target.m_leftLast = r.m_last;
  target.m_rightFirst = n - 1 - r.m_first;
  target.m_rightLast = n - 1 - r.m_last;
  target.m_distMeters = distMeters;
  return target;
}
}  // namespace

void MinimizeLaneChanges(std::vector<RouteSegment> & routeSegments, LaneChangeSettings const & settings)
{
  // Walk backwards: every junction prepares for the next junction with lane data or the next maneuver.
  Target target;

  for (auto it = routeSegments.rbegin(); it != routeSegments.rend(); ++it)
  {
    auto & segment = *it;
    auto const & turnItem = segment.GetTurn();
    auto & lanes = segment.GetTurnLanes();
    // Segments without an instruction still carry lanes of junctions passed straight on.
    if (turnItem.IsTurnNone() && lanes.empty())
      continue;

    auto const turn = turnItem.m_turn;
    double const dist = segment.GetDistFromBeginningMeters();
    bool const highway = IsHighway(segment);
    double const lookahead = highway ? settings.m_highwayLookaheadMeters : settings.m_cityLookaheadMeters;
    bool const targetInRange = target.m_kind != Target::Kind::None && target.m_distMeters - dist <= lookahead;

    if (!lanes.empty())
    {
      if (targetInRange && target.m_kind == Target::Kind::Lanes)
        KeepAlignedLane(lanes, target, settings.m_highwayPreferRight);
      else if (targetInRange && target.m_kind == Target::Kind::Side)
        KeepOneLane(lanes, target.m_side);
      else if (highway && (turnItem.IsTurnNone() || IsStayOnRoad(turn)))
        KeepOneLane(lanes, settings.m_highwayPreferRight ? Side::Right : Side::Left);

      auto const laneTarget = MakeLanesTarget(lanes, dist);
      if (laneTarget.m_kind != Target::Kind::None)
      {
        target = laneTarget;
        continue;
      }
    }

    // Update what the segments before this one should prepare for.
    Side const own = SideOf(turn);
    if (own != Side::None)
    {
      target = {};
      target.m_kind = Target::Kind::Side;
      target.m_side = own;
      target.m_distMeters = dist;
    }
    else if (!turnItem.IsTurnNone() && !IsStayOnRoad(turn))
    {
      // Roundabouts, destination etc. break the chain.
      target = {};
    }
  }
}

bool impl::SetRecommendedLaneWays(CarDirection const carDirection, LanesInfo & lanesInfo)
{
  LaneWay laneWay;
  switch (carDirection)
  {
  case CarDirection::GoStraight: laneWay = LaneWay::Through; break;
  case CarDirection::TurnRight: laneWay = LaneWay::Right; break;
  case CarDirection::TurnSharpRight: laneWay = LaneWay::SharpRight; break;
  case CarDirection::TurnSlightRight: [[fallthrough]];
  case CarDirection::ExitHighwayToRight: laneWay = LaneWay::SlightRight; break;
  case CarDirection::TurnLeft: laneWay = LaneWay::Left; break;
  case CarDirection::TurnSharpLeft: laneWay = LaneWay::SharpLeft; break;
  case CarDirection::TurnSlightLeft: [[fallthrough]];
  case CarDirection::ExitHighwayToLeft: laneWay = LaneWay::SlightLeft; break;
  case CarDirection::UTurnLeft: laneWay = LaneWay::ReverseLeft; break;
  case CarDirection::UTurnRight: laneWay = LaneWay::ReverseRight; break;
  default: return false;
  }

  bool isLaneConformed = false;
  for (auto & [laneWays, recommendedWay] : lanesInfo)
  {
    // Unrestricted lanes (blank/"none") should match "GoStraight" as a first-class lane recommendation
    if (laneWays.Contains(laneWay) || (carDirection == CarDirection::GoStraight && laneWays.IsUnrestricted()))
    {
      recommendedWay = laneWay;
      isLaneConformed = true;
    }
    FixRecommendedReverseLane(laneWays, recommendedWay);
  }
  return isLaneConformed;
}

bool impl::SetRecommendedLaneWaysApproximately(CarDirection const carDirection, LanesInfo & lanesInfo)
{
  std::vector<LaneWay> approximateLaneWays;
  switch (carDirection)
  {
  case CarDirection::UTurnLeft: approximateLaneWays = {LaneWay::SharpLeft}; break;
  case CarDirection::TurnSharpLeft: approximateLaneWays = {LaneWay::Left}; break;
  case CarDirection::TurnLeft: approximateLaneWays = {LaneWay::SlightLeft, LaneWay::SharpLeft}; break;
  case CarDirection::TurnSlightLeft: [[fallthrough]];
  case CarDirection::ExitHighwayToLeft: approximateLaneWays = {LaneWay::Left}; break;
  case CarDirection::GoStraight: approximateLaneWays = {LaneWay::SlightRight, LaneWay::SlightLeft}; break;
  case CarDirection::ExitHighwayToRight: [[fallthrough]];
  case CarDirection::TurnSlightRight: approximateLaneWays = {LaneWay::Right}; break;
  case CarDirection::TurnRight: approximateLaneWays = {LaneWay::SlightRight, LaneWay::SharpRight}; break;
  case CarDirection::TurnSharpRight: approximateLaneWays = {LaneWay::Right}; break;
  case CarDirection::UTurnRight: approximateLaneWays = {LaneWay::SharpRight}; break;
  default: return false;
  }

  bool isLaneConformed = false;
  for (auto & [laneWays, recommendedWay] : lanesInfo)
  {
    for (auto const & laneWay : approximateLaneWays)
    {
      if (laneWays.Contains(laneWay))
      {
        recommendedWay = laneWay;
        isLaneConformed = true;
        break;
      }
    }
  }

  return isLaneConformed;
}

bool impl::SetUnrestrictedLaneAsRecommended(CarDirection const carDirection, LanesInfo & lanesInfo)
{
  static auto constexpr setFirstUnrestrictedLane = [](LaneWay const laneWay, auto beginIt, auto endIt)
  {
    auto it = std::find_if(beginIt, endIt, [](auto const & laneInfo) { return laneInfo.laneWays.IsUnrestricted(); });
    if (it == endIt)
      return false;
    it->recommendedWay = laneWay;
    return true;
  };

  if (IsTurnMadeFromLeft(carDirection))
    return setFirstUnrestrictedLane(LaneWay::Left, lanesInfo.begin(), lanesInfo.end());
  if (IsTurnMadeFromRight(carDirection))
    return setFirstUnrestrictedLane(LaneWay::Right, lanesInfo.rbegin(), lanesInfo.rend());
  return false;
}
}  // namespace routing::turns::lanes
