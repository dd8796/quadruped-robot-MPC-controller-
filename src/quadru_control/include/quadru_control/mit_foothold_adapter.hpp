#pragma once
// Visual foothold adaptation (arXiv 2307.14775) for MIT locomotion: evaluate at lift-off, track
// the selected region until mid-swing, then freeze. MIT world z = world z - foot radius.
#include <array>
#include <cstdint>

#include <Eigen/Dense>

#include "cheetah_locomotion/robot_interface.h"
#include "grid_map_core/grid_map_core.hpp"
#include "quadru_control/foothold_safety.hpp"
#include "quadru_control/perceptive_planner.hpp"

namespace quadru_control
{

// foothold_adaptation: MIT nominal, nearest safe pixel ([8] in the paper), or convex region.
enum class FootholdMode : int { kNominal = 0, kHeuristic = 1, kConvexMpc = 2 };

class MitFootholdAdapter : public cheetah::FootholdAdapter
{
public:
  // Robot state for the current tick (world frame, URDF), set before locomotion.run().
  struct Context
  {
    const grid_map::GridMap * map = nullptr;
    const PerceptivePlanner::FootholdResults * results = nullptr;
    FootholdMode mode = FootholdMode::kConvexMpc;
    Eigen::Matrix3d trunkR = Eigen::Matrix3d::Identity();   // trunk -> world
    Eigen::Vector3d trunkOrigin = Eigen::Vector3d::Zero();  // trunk link origin, world
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();     // world
    Eigen::Matrix<double, 12, 1> q = Eigen::Matrix<double, 12, 1>::Zero();
  };

  struct Commit
  {
    bool valid = false;
    Eigen::Vector3d pointWorld = Eigen::Vector3d::Zero();   // foot center
    bool hasRegion = false;
    ConvexRegion region;
  };

  MitFootholdAdapter(PerceptivePlanner & planner, double footRadius, double liftHeight);

  void reset();
  void setContext(const Context & ctx) {ctx_ = ctx;}
  void adapt(int leg, float swingPhase, float swingTime, const Vec3<float> & pFoot, Vec3<float> & Pf) override;

  // Ground height under each foot (MIT world), for the estimator and the body height reference.
  const Vec4<float> & footGround() const {return footGround_;}
  // Footholds frozen this tick (visualization); cleared by the caller.
  std::array<Commit, 4> commits{};

private:
  struct Swing
  {
    bool active = false;
    bool requested = false;
    bool evaluated = false;
    bool hasTarget = false;
    bool convex = true;
    bool committed = false;
    ConvexRegion region;
    Eigen::Vector3d target = Eigen::Vector3d::Zero();   // MIT world, z = ground height
  };

  PerceptivePlanner & planner_;
  const double footRadius_;
  const double liftHeight_;
  Context ctx_;
  std::array<Swing, 4> swing_{};
  std::array<std::uint64_t, 4> swingId_{};
  Vec4<float> footGround_ = Vec4<float>::Zero();
};

}  // namespace quadru_control
