#pragma once
// Elevation map post-processing (hole filling + 3x3 median) and robot self-filter.
// Pure library (grid_map_core + Eigen), shared with quadru_control.
#include <vector>

#include <Eigen/Geometry>
#include <grid_map_core/grid_map_core.hpp>

namespace terrain_mapping
{

inline constexpr const char * kLayerElevation = "elevation";            // raw Kalman fusion
inline constexpr const char * kLayerFiltered = "elevation_filtered";    // hole-filled + median

struct TerrainProcessingParams
{
  int inpaintRadiusCells = 2;      // hole filled if enough valid neighbors within this radius
  int inpaintMinNeighbors = 3;
};

// Writes kLayerFiltered from kLayerElevation. The map must use the default start index.
void processTerrain(grid_map::GridMap & map, const TerrainProcessingParams & params = {});

// Robot self-filter shapes, in the map frame.
struct Capsule
{
  Eigen::Vector3f a = Eigen::Vector3f::Zero();
  Eigen::Vector3f b = Eigen::Vector3f::Zero();
  float radius = 0.f;
};
struct RobotShapes
{
  std::vector<Capsule> limbs;                                   // thighs and shanks
  Eigen::Isometry3f mapToTrunk = Eigen::Isometry3f::Identity(); // map -> trunk
  Eigen::Vector3f trunkBoxCenter = Eigen::Vector3f::Zero();     // trunk box center (trunk frame)
  Eigen::Vector3f trunkBoxHalf = Eigen::Vector3f::Zero();       // half extents
};
bool isRobotPoint(const Eigen::Vector3f & p, const RobotShapes & shapes);

}  // namespace terrain_mapping
