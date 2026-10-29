#pragma once
// Height queries on the elevation map.
#include <optional>

#include "grid_map_core/grid_map_core.hpp"

namespace quadru_control
{

// Elevation of the nearest observed cell (finite variance) within searchRadiusCells of (x, y);
// nullopt if none (the map is sparse).
std::optional<double> lookupTerrainHeight(
  const grid_map::GridMap & map, double xWorld, double yWorld, int searchRadiusCells);

}  // namespace quadru_control
