#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "quadru_control/perceptive_planner.hpp"
#include "quadru_control/terrain_awareness.hpp"

using namespace quadru_control;

namespace
{
const Eigen::Vector3d kQ(0.0, 0.52611, 1.11101567);          // standing pose (leg 0)
const Eigen::Vector3d kTrunk(0.0, 0.0, -0.20727654);

// Flat ground observed everywhere (elevation + variance layers), centered on leg 0.
std::shared_ptr<grid_map::GridMap> flatMapAround(const Eigen::Vector3d & foot)
{
  auto map = std::make_shared<grid_map::GridMap>(std::vector<std::string>{"elevation", "variance"});
  map->setGeometry(grid_map::Length(2.0, 1.0), 0.02, grid_map::Position(foot.x(), foot.y()));
  (*map)["elevation"].setZero();
  (*map)["variance"].setConstant(1e-4f);
  return map;
}

PerceptivePlanner::FootholdRequest requestFor(const Eigen::Vector3d & foot, std::uint64_t swingId)
{
  PerceptivePlanner::FootholdRequest req;
  req.swingId = swingId;
  req.legUrdf = 0;
  req.nominalW = Eigen::Vector2d(foot.x() - 0.05, foot.y());
  req.qLeg = kQ;
  req.trunkNow = kTrunk;
  req.trunkTouchdown = kTrunk + Eigen::Vector3d(-0.02, 0.0, 0.0);
  req.swingStart = foot;
  return req;
}
}  // namespace

// Posting a request never blocks, even during an evaluation.
TEST(PerceptivePlanner, RealtimePostNeverBlocksWhileEvaluating)
{
  const auto legs = buildAllLegs();
  PerceptivePlanner planner(legs, "elevation");
  const Eigen::Vector3d foot = legs[0].forwardKinematics(kQ) + kTrunk;
  planner.setTerrainMap(flatMapAround(foot));
  planner.start();
  double worstUs = 0.0;
  std::uint64_t posted = 0;
  const auto tEnd = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
  while (std::chrono::steady_clock::now() < tEnd) {
    const auto req = requestFor(foot, posted + 1);
    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = planner.postFootholdRequest(0, req);
    worstUs = std::max(worstUs, std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - t0).count());
    if (ok) {posted = req.swingId;}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));   // 1 kHz tick
  }
  planner.stop();
  planner.processPending();   // last request possibly still pending
  EXPECT_GT(posted, 100u);
  EXPECT_LT(worstUs, 500.0);   // keeps margin in a 1 ms tick
  EXPECT_EQ((*planner.readFootholdResults())[0].swingId, posted);   // latest request processed
}

// Results are tagged with the requesting swing (late results are detectable).
TEST(PerceptivePlanner, FootholdResultCarriesSwingIdAndConvexRegion)
{
  const auto legs = buildAllLegs();
  PerceptivePlanner planner(legs, "elevation");
  const Eigen::Vector3d foot = legs[0].forwardKinematics(kQ) + kTrunk;
  const auto map = flatMapAround(foot);
  planner.setTerrainMap(map);

  const auto req = requestFor(foot, 7);
  ASSERT_TRUE(planner.postFootholdRequest(2, req));
  planner.processPending();
  const auto & res = (*planner.readFootholdResults())[2];
  EXPECT_EQ(res.swingId, 7u);
  ASSERT_TRUE(res.hasTarget);
  EXPECT_NEAR(res.region.height, 0.0, 1e-6);
  EXPECT_NEAR(res.targetZ, req.params.footRadius, 1e-6);
  EXPECT_LT((res.targetW - req.nominalW).norm(), 1e-9);   // safe nominal unchanged
  EXPECT_EQ((*planner.readFootholdResults())[0].swingId, 0u);   // other legs untouched
  // Terrain height: observed cell.
  EXPECT_NEAR(*lookupTerrainHeight(*map, foot.x(), foot.y(), 3), 0.0, 1e-9);
  EXPECT_FALSE(lookupTerrainHeight(*map, foot.x() + 5.0, foot.y(), 3).has_value());
}
