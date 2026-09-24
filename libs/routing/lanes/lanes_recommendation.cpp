#include "lanes_recommendation.hpp"

#include "routing/lanes/lane_way.hpp"
#include "routing/route.hpp"
#include "routing/turns.hpp"

#include "indexer/ftypes_matcher.hpp"

#include <limits>

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

void SelectRecommendedLanes(std::vector<RouteSegment> & routeSegments)
{
  for (auto & segment : routeSegments)
  {
    auto & t = segment.GetTurn();
    if (t.IsTurnNone() || t.m_lanes.empty())
      continue;
    auto & lanesInfo = segment.GetTurnLanes();
    // Check if there are elements in lanesInfo that correspond with the turn exactly.
    // If so, fix up all the elements in lanesInfo that correspond with the turn.
    if (impl::SetRecommendedLaneWays(t.m_turn, lanesInfo))
      continue;
    // If not, check if there are elements in lanesInfo that correspond with the turn
    // approximately. If so, fix up all those elements.
    if (impl::SetRecommendedLaneWaysApproximately(t.m_turn, lanesInfo))
      continue;
    // If not, check if there is an unrestricted lane that could correspond to the
    // turn. If so, fix up that lane.
    if (impl::SetUnrestrictedLaneAsRecommended(t.m_turn, lanesInfo))
      continue;
    // Otherwise, we don't have lane recommendations for the user, so we don't
    // want to send the lane data any further.
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

// Keeps only the leftmost (or rightmost) recommended lane. Returns false if there is nothing to narrow.
bool KeepOneLane(LanesInfo & lanes, Side const side)
{
  size_t first = lanes.size(), last = lanes.size(), count = 0;
  for (size_t i = 0; i < lanes.size(); ++i)
  {
    if (lanes[i].recommendedWay == LaneWay::None)
      continue;
    if (first == lanes.size())
      first = i;
    last = i;
    ++count;
  }
  if (count < 2)
    return false;
  size_t const keep = side == Side::Left ? first : last;
  for (size_t i = 0; i < lanes.size(); ++i)
    if (i != keep)
      lanes[i].recommendedWay = LaneWay::None;
  return true;
}
}  // namespace

void MinimizeLaneChanges(std::vector<RouteSegment> & routeSegments, LaneChangeSettings const & settings)
{
  // Walk backwards: carry the side of the next left/right maneuver through straight junctions.
  Side pendingSide = Side::None;
  double pendingDist = std::numeric_limits<double>::max();

  for (auto it = routeSegments.rbegin(); it != routeSegments.rend(); ++it)
  {
    auto & segment = *it;
    auto const turn = segment.GetTurn().m_turn;
    if (segment.GetTurn().IsTurnNone())
      continue;

    double const dist = segment.GetDistFromBeginningMeters();
    bool const highway = IsHighway(segment);
    double const lookahead = highway ? settings.m_highwayLookaheadMeters : settings.m_cityLookaheadMeters;

    auto & lanes = segment.GetTurnLanes();
    if (!lanes.empty())
    {
      Side side = Side::None;
      if (pendingSide != Side::None && pendingDist - dist <= lookahead)
        side = pendingSide;
      else if (highway && IsStayOnRoad(turn))
        side = settings.m_highwayPreferRight ? Side::Right : Side::Left;
      if (side != Side::None)
        KeepOneLane(lanes, side);
    }

    // Update what the segments before this one should prepare for.
    Side const own = SideOf(turn);
    if (own != Side::None)
    {
      pendingSide = own;
      pendingDist = dist;
    }
    else if (!IsStayOnRoad(turn))
    {
      // Roundabouts, destination etc. break the chain.
      pendingSide = Side::None;
      pendingDist = std::numeric_limits<double>::max();
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
