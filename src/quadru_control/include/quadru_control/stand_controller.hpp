#pragma once
// Standing controller: weight support on the stance feet + joint PD to the captured pose
// (unitree_guide FixedStand).
#include <array>

#include <Eigen/Dense>

#include "quadru_control/leg_kinematics.hpp"
#include "quadru_control/robot_model.hpp"

namespace quadru_control
{

class StandController
{
public:
  StandController(const std::array<LegKinematics, 4> & legs, const RobotModel & model);

  void setTarget(const Eigen::Matrix<double, 12, 1> & qTarget) {qTarget_ = qTarget;}

  // Per leg: tau = G(q) - J^T R^T f + Kp (q* - q) - Kd qd, with f = (0, 0, M g / n) on the n
  // stance legs and 0 otherwise. assumeAllFeet: no contact reported -> assume all four.
  // R: trunk -> world.
  Eigen::Matrix<double, 12, 1> torques(
    const Eigen::Matrix<double, 12, 1> & q, const Eigen::Matrix<double, 12, 1> & qd,
    const std::array<bool, 4> & contactsUrdf, const Eigen::Matrix3d & R, bool assumeAllFeet) const;

private:
  const std::array<LegKinematics, 4> & legs_;
  const RobotModel & model_;
  Eigen::Matrix<double, 12, 1> qTarget_ = Eigen::Matrix<double, 12, 1>::Zero();
};

}  // namespace quadru_control
