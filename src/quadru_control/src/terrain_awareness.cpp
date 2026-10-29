#include "quadru_control/terrain_awareness.hpp"

#include <algorithm>
#include <cmath>
#include <limits>


namespace quadru_control
{

std::optional<double> lookupTerrainHeight(
  const grid_map::GridMap & map, double xWorld, double yWorld, int searchRadiusCells)
{
  if (!map.exists("elevation") || !map.exists("variance")) {return std::nullopt;}
  const grid_map::Position pos(xWorld, yWorld);
  if (!map.isInside(pos)) {return std::nullopt;}
  grid_map::Index idx;
  map.getIndex(pos, idx);
  const grid_map::Size & size = map.getSize();
  const grid_map::Matrix & variance = map["variance"];
  const grid_map::Matrix & elevation = map["elevation"];
  double bestDist2 = std::numeric_limits<double>::infinity();
  float best = std::numeric_limits<float>::quiet_NaN();
  for (int dr = -searchRadiusCells; dr <= searchRadiusCells; ++dr) {
    for (int dc = -searchRadiusCells; dc <= searchRadiusCells; ++dc) {
      const grid_map::Index cand(idx(0) + dr, idx(1) + dc);
      if (cand(0) < 0 || cand(0) >= size(0) || cand(1) < 0 || cand(1) >= size(1)) {continue;}
      if (std::isnan(variance(cand(0), cand(1)))) {continue;}
      const double dist2 = static_cast<double>(dr * dr + dc * dc);
      if (dist2 < bestDist2) {
        bestDist2 = dist2;
        best = elevation(cand(0), cand(1));
      }
    }
  }
  if (std::isnan(best)) {return std::nullopt;}
  return static_cast<double>(best);
}

}  // namespace quadru_control
