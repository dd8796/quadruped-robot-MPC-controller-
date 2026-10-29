#pragma once
// Foothold safety (II-A: reachability, roughness, frontal and leg collision) and convex
// decomposition (II-C) from Omar et al., arXiv 2307.14775. The criteria are computed directly
// (no SaFE-Net).
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <grid_map_core/grid_map_core.hpp>

#include "quadru_control/leg_kinematics.hpp"

namespace quadru_control
{

// Cell (i, j) is at world point origin + res * (i, j).
struct FootholdHeightmap
{
  Eigen::MatrixXd height;   // NaN = unknown
  Eigen::Vector2d origin = Eigen::Vector2d::Zero();
  double res = 0.02;
  Eigen::Vector2d cellXY(int i, int j) const {return origin + res * Eigen::Vector2d(i, j);}
};

FootholdHeightmap extractFootholdHeightmap(
  const grid_map::GridMap & map, const std::string & layer, const Eigen::Vector2d & nominalW,
  double halfSize);

// Positions in the world frame; leg kinematics in the URDF trunk frame.
struct LegContext
{
  const LegKinematics * leg = nullptr;
  Eigen::Vector3d qCurrent = Eigen::Vector3d::Zero();    // current joint angles (IK branch)
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();        // trunk -> world
  Eigen::Vector3d trunkNow = Eigen::Vector3d::Zero();     // trunk origin now
  Eigen::Vector3d trunkTouchdown = Eigen::Vector3d::Zero();  // trunk origin predicted at touchdown
  Eigen::Vector3d swingStart = Eigen::Vector3d::Zero();   // foot center at lift-off
};

struct SafetyParams
{
  double footRadius = 0.02;
  // (b) roughness
  int roughnessRadiusCells = 2;
  double roughnessHeightThreshold = 0.02;
  int roughnessMinNeighbors = 2;
  // (a) reachability
  double ikTol = 3e-3;
  double minKneeAbs = 0.25;          // rad, rejects straight legs and knee flips
  // (c) frontal collision, along the MIT Bezier swing actually executed
  double swingDuration = 0.144;
  double liftHeight = 0.08;
  int swingSamples = 12;
  double collisionTol = 0.005;
  // (d) leg collision
  int limbSamples = 4;
  double limbClearance = 0.01;
};

struct CriteriaMask
{
  bool reachability = true, roughness = true, frontalCollision = true, legCollision = true;
};

// Rejected pixels per criterion (first failing criterion only).
struct SafetyStats
{
  int unknown = 0, roughness = 0, reachability = 0, frontalCollision = 0, legCollision = 0;
};

// Safety map (1 = safe) of the heightmap. map/layer are used for the collision checks along
// the swing, which leave the foothold window.
Eigen::MatrixXi evaluateFootholdSafety(
  const FootholdHeightmap & hm, const grid_map::GridMap & map, const std::string & layer,
  const LegContext & ctx, const SafetyParams & params = {}, const CriteriaMask & criteria = {},
  SafetyStats * stats = nullptr);

// Axis-aligned rectangle in world coordinates; bounds are cell centers.
struct ConvexRegion
{
  Eigen::Vector2d min = Eigen::Vector2d::Zero();
  Eigen::Vector2d max = Eigen::Vector2d::Zero();
  double height = 0.0;   // median cell height
  int cells = 0;
};

// Greedy split into non-overlapping maximal rectangles, largest first.
std::vector<ConvexRegion> decomposeConvexRegions(
  const FootholdHeightmap & hm, const Eigen::MatrixXi & safe, int minCells = 4, int maxRegions = 8);

// Nearest region to the nominal foothold (ties: largest).
std::optional<ConvexRegion> selectNearestRegion(
  const std::vector<ConvexRegion> & regions, const Eigen::Vector2d & nominalW);

// Baseline [8] of the paper (Villarreal et al. 2020): nearest safe point to the nominal.
std::optional<Eigen::Vector3d> nearestSafeFoothold(
  const FootholdHeightmap & hm, const Eigen::MatrixXi & safe, const Eigen::Vector2d & nominalW);

}  // namespace quadru_control
