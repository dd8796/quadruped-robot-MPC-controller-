// Adapted MIT MPC: standing support, no force on swing legs, solve time.
#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "ConvexMPCLocomotion.h"
#include "Math/orientation_tools.h"

using namespace cheetah;

namespace
{
RobotParams quadruParams()
{
  RobotParams r;
  r.mass = 14.58f;
  r.inertia = Vec3<float>(0.12f, 0.35f, 0.40f).asDiagonal();
  const float hx = 0.29f, hy = 0.09f;
  const float sx[4] = {1, 1, -1, -1}, sy[4] = {-1, 1, -1, 1};
  for (int i = 0; i < 4; ++i) {
    r.hipLocation[i] = Vec3<float>(sx[i] * hx, sy[i] * hy, 0.f);
    r.nominalFoot[i] = Vec3<float>(sx[i] * hx, sy[i] * 0.12f, -0.3f);
  }
  r.bodyHeight = 0.3f;
  r.fMax = 150.f;
  return r;
}

StateEstimate<float> standingState(const RobotParams & r)
{
  StateEstimate<float> se;
  se.position = Vec3<float>(0, 0, r.bodyHeight);
  return se;
}

void standingLegs(const RobotParams & r, LegData legs[4])
{
  for (int i = 0; i < 4; ++i) {
    legs[i].p = Vec3<float>(r.nominalFoot[i][0], r.nominalFoot[i][1], -r.bodyHeight) - r.hipLocation[i];
    legs[i].v.setZero();
  }
}
}  // namespace

TEST(CheetahLocomotion, StandingSupportsWeight)
{
  const RobotParams r = quadruParams();
  ConvexMPCLocomotion mpc(0.001f, 27, r);
  LegData legs[4];
  standingLegs(r, legs);
  const StateEstimate<float> se = standingState(r);
  LocomotionCommand cmd;
  cmd.stand = true;
  LegCommand out[4];
  Vec4<float> phase;
  for (int k = 0; k < 27 * 3; ++k) {mpc.run(se, legs, cmd, out, phase);}
  ASSERT_TRUE(mpc.lastSolveOk);
  float fz = 0.f;
  for (int i = 0; i < 4; ++i) {fz += mpc.Fr_des[i][2];}
  // The original MIT MPC carries 8.5 % more than the weight at rest (alpha regularization).
  EXPECT_NEAR(fz, r.mass * 9.81f, 0.1f * r.mass * 9.81f);
  for (int i = 0; i < 4; ++i) {EXPECT_NEAR(mpc.Fr_des[i][2], fz / 4.f, 1e-2f * fz);}
}

TEST(CheetahLocomotion, TrotSwingLegsGetNoForceAndSolveIsFast)
{
  const RobotParams r = quadruParams();
  ConvexMPCLocomotion mpc(0.001f, 27, r);
  LegData legs[4];
  standingLegs(r, legs);
  const StateEstimate<float> se = standingState(r);
  LocomotionCommand cmd;
  cmd.vx = 0.3f;
  LegCommand out[4];
  Vec4<float> phase;
  std::vector<float> solveMs;
  int swingChecks = 0;
  for (int k = 0; k < 27 * 40; ++k) {
    mpc.run(se, legs, cmd, out, phase);
    if (k % 27 == 26) {   // MIT increments the counter before solving
      ASSERT_TRUE(mpc.lastSolveOk) << "k=" << k;
      solveMs.push_back(mpc.lastSolveMs);
    }
    for (int i = 0; i < 4; ++i) {
      if (phase[i] == 0.f && out[i].kpCartesian(0, 0) > 0.f) {   // swing leg
        EXPECT_EQ(out[i].forceFeedForward.norm(), 0.f);
        ++swingChecks;
      }
    }
  }
  EXPECT_GT(swingChecks, 0);
  std::sort(solveMs.begin(), solveMs.end());
  const float median = solveMs[solveMs.size() / 2];
  RecordProperty("solve_ms_median", std::to_string(median));
  std::printf("[ temps QP ] median %.3f ms, max %.3f ms (%zu QP)\n", median, solveMs.back(), solveMs.size());
  EXPECT_LT(median, 5.f);
}
