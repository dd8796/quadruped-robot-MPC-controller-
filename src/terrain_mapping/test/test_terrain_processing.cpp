#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "terrain_mapping/terrain_processing.hpp"

using terrain_mapping::processTerrain;

namespace
{
constexpr double kStepHeight = 0.05;

// 2 x 1 m map, 2 cm cells. Ground at 0 for x >= 0, step top at kStepHeight for x < 0.
grid_map::GridMap makeStepMap(double noise = 0.0)
{
  grid_map::GridMap map({terrain_mapping::kLayerElevation});
  map.setGeometry(grid_map::Length(2.0, 1.0), 0.02, grid_map::Position(0.0, 0.0));
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> u(-noise, noise);
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Position pos;
    map.getPosition(*it, pos);
    map.at(terrain_mapping::kLayerElevation, *it) =
      static_cast<float>((pos.x() < 0.0 ? kStepHeight : 0.0) + (noise > 0 ? u(rng) : 0.0));
  }
  return map;
}

float atPos(const grid_map::GridMap & map, const char * layer, double x, double y)
{
  return map.atPosition(layer, grid_map::Position(x, y));
}
}  // namespace

TEST(TerrainProcessing, HoleIsInpainted)
{
  auto map = makeStepMap();
  for (double dx = -0.02; dx <= 0.021; dx += 0.02) {
    for (double dy = -0.02; dy <= 0.021; dy += 0.02) {
      map.atPosition(terrain_mapping::kLayerElevation, grid_map::Position(0.41 + dx, 0.21 + dy)) =
        std::numeric_limits<float>::quiet_NaN();
    }
  }
  processTerrain(map);
  const float h = atPos(map, terrain_mapping::kLayerFiltered, 0.41, 0.21);
  ASSERT_TRUE(std::isfinite(h));
  EXPECT_NEAR(h, 0.0, 1e-3);
}

TEST(TerrainProcessing, IsolatedFlyingPixelIsRemovedEdgeIsKept)
{
  auto map = makeStepMap();
  map.atPosition(terrain_mapping::kLayerElevation, grid_map::Position(0.41, 0.21)) = 0.30f;
  processTerrain(map);
  EXPECT_NEAR(atPos(map, terrain_mapping::kLayerFiltered, 0.41, 0.21), 0.0, 1e-6);
  // The step edge is a real structure: the median keeps it.
  EXPECT_NEAR(atPos(map, terrain_mapping::kLayerFiltered, -0.05, 0.0), kStepHeight, 1e-6);
  EXPECT_NEAR(atPos(map, terrain_mapping::kLayerFiltered, 0.05, 0.0), 0.0, 1e-6);
}

TEST(TerrainProcessing, RobotSelfFilterRejectsLimbAndTrunkPointsOnly)
{
  terrain_mapping::RobotShapes shapes;
  terrain_mapping::Capsule shank;
  shank.a = Eigen::Vector3f(0.f, 0.f, 0.3f);    // knee
  shank.b = Eigen::Vector3f(0.1f, 0.f, 0.02f);  // foot
  shank.radius = 0.04f;
  shapes.limbs.push_back(shank);
  shapes.trunkBoxCenter = Eigen::Vector3f(0.f, 0.f, 0.5f);
  shapes.trunkBoxHalf = Eigen::Vector3f(0.3f, 0.15f, 0.1f);
  EXPECT_TRUE(terrain_mapping::isRobotPoint(Eigen::Vector3f(0.05f, 0.02f, 0.16f), shapes));   // on the shank
  EXPECT_TRUE(terrain_mapping::isRobotPoint(Eigen::Vector3f(0.2f, 0.1f, 0.55f), shapes));     // inside the trunk
  EXPECT_FALSE(terrain_mapping::isRobotPoint(Eigen::Vector3f(0.25f, 0.f, 0.0f), shapes));     // ground ahead of the foot
  EXPECT_FALSE(terrain_mapping::isRobotPoint(Eigen::Vector3f(0.05f, 0.15f, 0.16f), shapes));  // next to the leg
}
