#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "quadru_control/foothold_safety.hpp"

using namespace quadru_control;

namespace
{
// Scene: front-left leg (URDF leg1) in walking stance. Forward is -x. Riser of kStep 10 cm
// ahead of the foot.
constexpr double kStep = 0.05;
constexpr double kFootRadiusTest = 0.02;   // SafetyParams::footRadius default

struct Scene
{
  std::array<LegKinematics, 4> legs = buildAllLegs();
  Eigen::Vector3d q{0.0, 0.52611, 1.11101567};
  Eigen::Vector3d trunk{0.0, 0.0, -0.20727654};
  Eigen::Vector3d foot;
  double edgeX;
  grid_map::GridMap map{{"elevation"}};
  LegContext ctx;
  Eigen::Vector2d nominal;

  explicit Scene(double wallHeight = kStep, double edgeAhead = 0.10)
  {
    foot = legs[0].forwardKinematics(q) + trunk;
    edgeX = foot.x() - edgeAhead;
    map.setGeometry(grid_map::Length(2.0, 1.0), 0.02, grid_map::Position(foot.x(), foot.y()));
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      grid_map::Position p;
      map.getPosition(*it, p);
      map.at("elevation", *it) = static_cast<float>(p.x() < edgeX ? wallHeight : 0.0);
    }
    ctx.leg = &legs[0];
    ctx.qCurrent = q;
    ctx.R.setIdentity();
    ctx.trunkNow = trunk;
    ctx.trunkTouchdown = trunk + Eigen::Vector3d(-0.02, 0.0, 0.0);
    ctx.swingStart = foot;
    nominal = Eigen::Vector2d(foot.x() - 0.15, foot.y());   // 5 cm past the edge
  }
  // Safety map value at the pixel nearest to (x, y).
  int safeAt(const FootholdHeightmap & hm, const Eigen::MatrixXi & safe, double x, double y) const
  {
    const Eigen::Vector2d ij = (Eigen::Vector2d(x, y) - hm.origin) / hm.res;
    return safe(static_cast<int>(std::lround(ij.x())), static_cast<int>(std::lround(ij.y())));
  }
};

CriteriaMask only(bool reach, bool rough, bool frontal, bool leg)
{
  CriteriaMask c;
  c.reachability = reach; c.roughness = rough; c.frontalCollision = frontal; c.legCollision = leg;
  return c;
}
}  // namespace

TEST(FootholdSafety, RoughnessRejectsTheEdgeOnlyAroundTheRiser)
{
  Scene s;
  const auto hm = extractFootholdHeightmap(s.map, "elevation", s.nominal, 0.12);
  const auto safe = evaluateFootholdSafety(hm, s.map, "elevation", s.ctx, {}, only(false, true, false, false));
  EXPECT_EQ(s.safeAt(hm, safe, s.edgeX - 0.01, s.foot.y()), 0);   // on the edge (top side)
  EXPECT_EQ(s.safeAt(hm, safe, s.edgeX + 0.01, s.foot.y()), 0);   // at the riser foot
  EXPECT_EQ(s.safeAt(hm, safe, s.edgeX - 0.09, s.foot.y()), 1);   // middle of the tread
  EXPECT_EQ(s.safeAt(hm, safe, s.edgeX + 0.09, s.foot.y()), 1);   // ground, far from the edge
}

TEST(FootholdSafety, ReachabilityRejectsPixelsOutsideLegWorkspace)
{
  Scene s(0.0);   // flat ground: kinematics only
  const Eigen::Vector2d farNominal(s.foot.x() - 0.45, s.foot.y());
  const auto hm = extractFootholdHeightmap(s.map, "elevation", farNominal, 0.04);
  const auto safe = evaluateFootholdSafety(hm, s.map, "elevation", s.ctx, {}, only(true, false, false, false));
  EXPECT_EQ(safe.sum(), 0);   // 45 cm ahead, trunk barely moving: out of reach

  const auto hmNear = extractFootholdHeightmap(s.map, "elevation", s.nominal, 0.04);
  const auto safeNear = evaluateFootholdSafety(hmNear, s.map, "elevation", s.ctx, {}, only(true, false, false, false));
  EXPECT_EQ(s.safeAt(hmNear, safeNear, s.nominal.x(), s.nominal.y()), 1);
}

TEST(FootholdSafety, FrontalCollisionDependsOnTheActualSwingTrajectory)
{
  // Flat ground, narrow obstacle half-way. The swing reaches ~1.6 x 8 cm mid-way: a 3 cm
  // obstacle is cleared, a 15 cm one is not.
  auto withRidge = [](double ridgeHeight) {
      Scene s(0.0);
      const double xr = s.foot.x() - 0.075;
      for (grid_map::GridMapIterator it(s.map); !it.isPastEnd(); ++it) {
        grid_map::Position p;
        s.map.getPosition(*it, p);
        if (std::abs(p.x() - xr) < 0.02) {s.map.at("elevation", *it) = static_cast<float>(ridgeHeight);}
      }
      return s;
    };
  for (const double h : {0.03, 0.15}) {
    Scene s = withRidge(h);
    s.ctx.leg = &s.legs[0];   // must not point to the temporary Scene's legs
    const auto hm = extractFootholdHeightmap(s.map, "elevation", s.nominal, 0.04);
    const auto safe = evaluateFootholdSafety(hm, s.map, "elevation", s.ctx, {}, only(false, false, true, false));
    EXPECT_EQ(s.safeAt(hm, safe, s.nominal.x(), s.nominal.y()), h < 0.1 ? 1 : 0) << "obstacle " << h;
  }
}

TEST(FootholdSafety, LegCollisionHasNoFalsePositiveOnFlatGround)
{
  Scene s(0.0);
  const auto hm = extractFootholdHeightmap(s.map, "elevation", s.nominal, 0.04);
  const auto safe = evaluateFootholdSafety(hm, s.map, "elevation", s.ctx, {}, only(false, false, false, true));
  EXPECT_EQ(s.safeAt(hm, safe, s.nominal.x(), s.nominal.y()), 1);
}

TEST(FootholdSafety, ConvexRegionsContainOnlySafeCellsAndNearestIsOnTheTread)
{
  Scene s;
  const auto hm = extractFootholdHeightmap(s.map, "elevation", s.nominal, 0.12);
  const auto safe = evaluateFootholdSafety(hm, s.map, "elevation", s.ctx);
  const auto regions = decomposeConvexRegions(hm, safe);
  ASSERT_FALSE(regions.empty());
  for (const auto & reg : regions) {
    for (double x = reg.min.x(); x <= reg.max.x() + 1e-9; x += hm.res) {
      for (double y = reg.min.y(); y <= reg.max.y() + 1e-9; y += hm.res) {
        EXPECT_EQ(s.safeAt(hm, safe, x, y), 1);
      }
    }
  }
  // Nominal 5 cm past the edge (rejected band): the region must be on the tread, off the edge.
  const auto chosen = selectNearestRegion(regions, s.nominal);
  ASSERT_TRUE(chosen.has_value());
  EXPECT_NEAR(chosen->height, kStep, 1e-6);
  EXPECT_LT(chosen->max.x(), s.edgeX);
}

TEST(FootholdSafety, HeuristicBaselineReturnsNearestSafePixel)
{
  Scene s;
  const auto hm = extractFootholdHeightmap(s.map, "elevation", s.nominal, 0.12);
  const auto safe = evaluateFootholdSafety(hm, s.map, "elevation", s.ctx);
  const auto f = nearestSafeFoothold(hm, safe, s.nominal);
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(s.safeAt(hm, safe, f->x(), f->y()), 1);
  EXPECT_LT((f->head<2>() - s.nominal).norm(), 0.10);
}

TEST(FootholdSafety, HeuristicBaselineKeepsSafeNominalUnchanged)
{
  // Flat ground: the nominal is safe and [8] keeps it (no snapping to the pixel center).
  Scene s(kStep, 0.60);
  const Eigen::Vector2d nominal = s.nominal + Eigen::Vector2d(0.0037, -0.0061);
  const auto hm = extractFootholdHeightmap(s.map, "elevation", nominal, 0.12);
  const auto safe = evaluateFootholdSafety(hm, s.map, "elevation", s.ctx);
  ASSERT_EQ(s.safeAt(hm, safe, nominal.x(), nominal.y()), 1);
  const auto f = nearestSafeFoothold(hm, safe, nominal);
  ASSERT_TRUE(f.has_value());
  EXPECT_LT((f->head<2>() - nominal).norm(), 1e-9);
}

TEST(FootholdSafety, FootRestingAgainstRiserIsNotAFrontalCollision)
{
  // Foot resting against a 5 cm riser, target on the step: the swing rises first, no collision.
  Scene s(kStep, kFootRadiusTest + 0.002);
  const auto hm = extractFootholdHeightmap(s.map, "elevation", s.nominal, 0.04);
  const auto safe = evaluateFootholdSafety(hm, s.map, "elevation", s.ctx, {}, only(false, false, true, false));
  EXPECT_EQ(s.safeAt(hm, safe, s.nominal.x(), s.nominal.y()), 1);
}
