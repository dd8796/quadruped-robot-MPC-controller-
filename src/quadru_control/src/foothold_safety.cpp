#include "quadru_control/foothold_safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stack>

#include "cheetah_locomotion/FootSwingTrajectory.h"

namespace quadru_control
{

namespace
{
std::optional<double> terrainAt(
  const grid_map::GridMap & map, const std::string & layer, double x, double y)
{
  const grid_map::Position pos(x, y);
  if (!map.isInside(pos)) {return std::nullopt;}
  const float h = map.atPosition(layer, pos);
  if (!std::isfinite(h)) {return std::nullopt;}
  return static_cast<double>(h);
}

// Highest terrain under the foot footprint (center + 8 points at radius r).
std::optional<double> terrainUnderFoot(
  const grid_map::GridMap & map, const std::string & layer, double x, double y, double r)
{
  std::optional<double> hmax = terrainAt(map, layer, x, y);
  for (int k = 0; k < 8; ++k) {
    const double a = k * M_PI / 4.0;
    const auto h = terrainAt(map, layer, x + r * std::cos(a), y + r * std::sin(a));
    if (h && (!hmax || *h > *hmax)) {hmax = h;}
  }
  return hmax;
}

// Does the foot sphere penetrate the terrain? Check below the center and at half radius;
// a foot resting against a riser (tangent contact) is not a collision.
bool sphereHitsTerrain(
  const grid_map::GridMap & map, const std::string & layer, const Eigen::Vector3d & c, double r,
  double tol)
{
  if (const auto h = terrainAt(map, layer, c.x(), c.y()); h && *h > c.z() - r + tol) {return true;}
  const double rho = 0.5 * r;
  const double bottom = c.z() - r * std::sqrt(3.0) / 2.0;
  for (int k = 0; k < 8; ++k) {
    const double a = k * M_PI / 4.0;
    const auto h = terrainAt(map, layer, c.x() + rho * std::cos(a), c.y() + rho * std::sin(a));
    if (h && *h > bottom + tol) {return true;}
  }
  return false;
}

// Thigh and shank above the terrain? The last 20 % of the shank may touch the ground.
bool limbsClearTerrain(
  const std::array<Eigen::Vector3d, 4> & ptsTrunk, const Eigen::Matrix3d & R,
  const Eigen::Vector3d & trunk, const grid_map::GridMap & map, const std::string & layer,
  const SafetyParams & p)
{
  const Eigen::Vector3d flex = R * ptsTrunk[1] + trunk;
  const Eigen::Vector3d knee = R * ptsTrunk[2] + trunk;
  const Eigen::Vector3d foot = R * ptsTrunk[3] + trunk;
  auto segmentClear = [&](const Eigen::Vector3d & a, const Eigen::Vector3d & b, double tMax) {
      for (int k = 0; k <= p.limbSamples; ++k) {
        const double t = tMax * static_cast<double>(k) / p.limbSamples;
        const Eigen::Vector3d x = a + t * (b - a);
        const auto h = terrainAt(map, layer, x.x(), x.y());
        if (h && x.z() < *h + p.limbClearance) {return false;}
      }
      return true;
    };
  return segmentClear(flex, knee, 1.0) && segmentClear(knee, foot, 0.8);
}
}  // namespace

FootholdHeightmap extractFootholdHeightmap(
  const grid_map::GridMap & map, const std::string & layer, const Eigen::Vector2d & nominalW,
  double halfSize)
{
  FootholdHeightmap hm;
  hm.res = map.getResolution();
  const int half = static_cast<int>(std::round(halfSize / hm.res));
  const int n = 2 * half + 1;
  hm.origin = nominalW - hm.res * Eigen::Vector2d(half, half);
  hm.height.setConstant(n, n, std::numeric_limits<double>::quiet_NaN());
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const Eigen::Vector2d xy = hm.cellXY(i, j);
      if (const auto h = terrainAt(map, layer, xy.x(), xy.y())) {hm.height(i, j) = *h;}
    }
  }
  return hm;
}

namespace
{
Eigen::Vector3d swingPosition(
  const Eigen::Vector3d & start, const Eigen::Vector3d & target, double s, const SafetyParams & p)
{
  FootSwingTrajectory<double> traj;
  traj.setHeight(p.liftHeight);
  traj.setInitialPosition(start);
  traj.setFinalPosition(target);
  traj.computeSwingTrajectoryBezier(std::clamp(s, 0.0, 1.0), p.swingDuration);
  return traj.getPosition();
}
}  // namespace

Eigen::MatrixXi evaluateFootholdSafety(
  const FootholdHeightmap & hm, const grid_map::GridMap & map, const std::string & layer,
  const LegContext & ctx, const SafetyParams & p, const CriteriaMask & crit, SafetyStats * stats)
{
  SafetyStats local;
  SafetyStats & st = stats ? *stats : local;
  st = SafetyStats{};
  const int n0 = static_cast<int>(hm.height.rows());
  const int n1 = static_cast<int>(hm.height.cols());
  Eigen::MatrixXi safe = Eigen::MatrixXi::Zero(n0, n1);
  const double kneeSign = ctx.qCurrent(2) >= 0.0 ? 1.0 : -1.0;
  const int r = p.roughnessRadiusCells;

  for (int i = 0; i < n0; ++i) {
    for (int j = 0; j < n1; ++j) {
      const double h = hm.height(i, j);
      if (!std::isfinite(h)) {++st.unknown; continue;}   // unknown terrain is never safe

      // (b) roughness: neighbors (unknown included) beyond the height threshold.
      if (crit.roughness) {
        int bad = 0;
        for (int di = -r; di <= r; ++di) {
          for (int dj = -r; dj <= r; ++dj) {
            if (di == 0 && dj == 0) {continue;}
            const int a = i + di, b = j + dj;
            if (a < 0 || b < 0 || a >= n0 || b >= n1) {continue;}
            const double hn = hm.height(a, b);
            if (!std::isfinite(hn) || std::abs(hn - h) > p.roughnessHeightThreshold) {++bad;}
          }
        }
        if (bad >= p.roughnessMinNeighbors) {++st.roughness; continue;}
      }

      const Eigen::Vector2d xy = hm.cellXY(i, j);
      const Eigen::Vector3d target(xy.x(), xy.y(), h + p.footRadius);

      // (a) reachability: IK at the trunk pose predicted at touchdown, same knee branch.
      Eigen::Vector3d qTd = ctx.qCurrent;
      if (crit.reachability || crit.legCollision) {
        const Eigen::Vector3d tTrunk = ctx.R.transpose() * (target - ctx.trunkTouchdown);
        const bool ok = ctx.leg->inverseKinematics(tTrunk, ctx.qCurrent, qTd, p.ikTol);
        if (crit.reachability &&
          (!ok || std::abs(qTd(2)) < p.minKneeAbs || qTd(2) * kneeSign < 0.0))
        {
          ++st.reachability;
          continue;
        }
      }

      // (c) frontal collision along the actual swing trajectory.
      bool collides = false;
      if (crit.frontalCollision) {
        for (int k = 1; k < p.swingSamples && !collides; ++k) {
          const double s = 0.9 * static_cast<double>(k) / p.swingSamples;
          const Eigen::Vector3d pos = swingPosition(ctx.swingStart, target, s, p);
          if (sphereHitsTerrain(map, layer, pos, p.footRadius, p.collisionTol)) {collides = true;}
        }
        if (collides) {++st.frontalCollision; continue;}
      }

      // (d) leg collision at touchdown and at 3 swing instants.
      if (crit.legCollision) {
        if (!limbsClearTerrain(ctx.leg->chainPoints(qTd), ctx.R, ctx.trunkTouchdown, map, layer, p)) {
          ++st.legCollision;
          continue;
        }
        Eigen::Vector3d q = ctx.qCurrent;
        for (const double s : {0.25, 0.5, 0.75}) {
          const Eigen::Vector3d pos = swingPosition(ctx.swingStart, target, s, p);
          const Eigen::Vector3d trunk = ctx.trunkNow + s * (ctx.trunkTouchdown - ctx.trunkNow);
          ctx.leg->inverseKinematics(ctx.R.transpose() * (pos - trunk), q, q, p.ikTol);
          if (!limbsClearTerrain(ctx.leg->chainPoints(q), ctx.R, trunk, map, layer, p)) {
            collides = true;
            break;
          }
        }
        if (collides) {++st.legCollision; continue;}
      }
      safe(i, j) = 1;
    }
  }
  return safe;
}

std::vector<ConvexRegion> decomposeConvexRegions(
  const FootholdHeightmap & hm, const Eigen::MatrixXi & safeIn, int minCells, int maxRegions)
{
  Eigen::MatrixXi safe = safeIn;
  const int n0 = static_cast<int>(safe.rows());
  const int n1 = static_cast<int>(safe.cols());
  std::vector<ConvexRegion> regions;
  while (static_cast<int>(regions.size()) < maxRegions) {
    // Largest all-ones rectangle: column histograms + monotonic stack.
    int bestArea = 0, bi0 = 0, bi1 = 0, bj0 = 0, bj1 = 0;
    Eigen::VectorXi colHeight = Eigen::VectorXi::Zero(n1);
    for (int i = 0; i < n0; ++i) {
      for (int j = 0; j < n1; ++j) {colHeight(j) = safe(i, j) ? colHeight(j) + 1 : 0;}
      std::stack<int> st;
      for (int j = 0; j <= n1; ++j) {
        const int hcur = (j == n1) ? 0 : colHeight(j);
        while (!st.empty() && colHeight(st.top()) >= hcur) {
          const int top = st.top();
          st.pop();
          const int height = colHeight(top);
          const int left = st.empty() ? 0 : st.top() + 1;
          const int area = height * (j - left);
          if (area > bestArea) {
            bestArea = area;
            bi0 = i - height + 1; bi1 = i; bj0 = left; bj1 = j - 1;
          }
        }
        if (j < n1) {st.push(j);}
      }
    }
    if (bestArea < minCells) {break;}
    std::vector<double> hs;
    hs.reserve(bestArea);
    for (int i = bi0; i <= bi1; ++i) {
      for (int j = bj0; j <= bj1; ++j) {hs.push_back(hm.height(i, j)); safe(i, j) = 0;}
    }
    std::nth_element(hs.begin(), hs.begin() + hs.size() / 2, hs.end());
    ConvexRegion reg;
    reg.min = hm.cellXY(bi0, bj0);
    reg.max = hm.cellXY(bi1, bj1);
    reg.height = hs[hs.size() / 2];
    reg.cells = bestArea;
    regions.push_back(reg);
  }
  return regions;
}

std::optional<ConvexRegion> selectNearestRegion(
  const std::vector<ConvexRegion> & regions, const Eigen::Vector2d & nominalW)
{
  std::optional<ConvexRegion> best;
  double bestDist = std::numeric_limits<double>::infinity();
  for (const auto & reg : regions) {
    const Eigen::Vector2d clamped = nominalW.cwiseMax(reg.min).cwiseMin(reg.max);
    const double d = (nominalW - clamped).norm();
    if (d < bestDist - 1e-9 || (std::abs(d - bestDist) <= 1e-9 && best && reg.cells > best->cells)) {
      bestDist = d;
      best = reg;
    }
  }
  return best;
}

std::optional<Eigen::Vector3d> nearestSafeFoothold(
  const FootholdHeightmap & hm, const Eigen::MatrixXi & safe, const Eigen::Vector2d & nominalW)
{
  std::optional<Eigen::Vector3d> best;
  double bestD2 = std::numeric_limits<double>::infinity();
  for (int i = 0; i < safe.rows(); ++i) {
    for (int j = 0; j < safe.cols(); ++j) {
      if (!safe(i, j)) {continue;}
      // Point of the cell nearest to the nominal: keeps a safe nominal unchanged and the target
      // continuous as the nominal moves.
      const Eigen::Vector2d c = hm.cellXY(i, j);
      const double h = 0.499 * hm.res;
      const Eigen::Vector2d xy = nominalW.cwiseMax((c.array() - h).matrix()).cwiseMin((c.array() + h).matrix());
      const double d2 = (xy - nominalW).squaredNorm();
      if (d2 < bestD2) {bestD2 = d2; best = Eigen::Vector3d(xy.x(), xy.y(), hm.height(i, j));}
    }
  }
  return best;
}

}  // namespace quadru_control
