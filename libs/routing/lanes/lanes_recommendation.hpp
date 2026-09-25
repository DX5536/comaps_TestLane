#pragma once

#include "routing/lanes/lane_info.hpp"

#include <vector>

namespace routing
{
class RouteSegment;

namespace turns
{
enum class CarDirection;
}  // namespace turns
}  // namespace routing

namespace routing::turns::lanes
{
/// Tuning for the "minimal lane change" pass.
struct LaneChangeSettings
{
  // How far ahead (meters) to look for the next maneuver in the city / on highways.
  double m_cityLookaheadMeters = 1000.0;
  double m_highwayLookaheadMeters = 3000.0;
  // On motorway/trunk with no upcoming maneuver, recommend the rightmost lane
  // (German "Rechtsfahrgebot", StVO §7). Set false to recommend the leftmost lane instead.
  bool m_highwayPreferRight = true;
  // Disable to get the stock CoMaps behaviour.
  bool m_enabled = true;
};

LaneChangeSettings & GetLaneChangeSettings();

/// Marks the lanes of |lanesInfo| which lead to |carDirection|.
/// \returns false if no lane matches, in which case the lanes should not be shown.
bool SetRecommendedLanes(CarDirection carDirection, LanesInfo & lanesInfo);

/// Selects lanes which are recommended for an end user.
/// Lanes of junctions passed without an instruction (turn is None) are expected to be already recommended.
void SelectRecommendedLanes(std::vector<RouteSegment> & routeSegments);

/// Narrows multiple recommended lanes down to one. Walks the route backwards and picks, at every
/// junction with lane data, the lane that lines up with the lane needed at the next junction
/// (or with the side of the next maneuver), so the driver doesn't have to cross lanes later.
void MinimizeLaneChanges(std::vector<RouteSegment> & routeSegments, LaneChangeSettings const & settings);

// Keep signatures in the header for testing purposes
namespace impl
{
bool SetRecommendedLaneWays(CarDirection carDirection, LanesInfo & lanesInfo);

bool SetRecommendedLaneWaysApproximately(CarDirection carDirection, LanesInfo & lanesInfo);

bool SetUnrestrictedLaneAsRecommended(CarDirection carDirection, LanesInfo & lanesInfo);
}  // namespace impl
}  // namespace routing::turns::lanes
