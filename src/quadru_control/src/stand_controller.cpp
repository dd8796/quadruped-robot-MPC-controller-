#include "quadru_control/stand_controller.hpp"

#include <algorithm>

#include "quadru_control/controller_config.hpp"

namespace quadru_control
{

StandController::StandController(const std::array<LegKinematics, 4> & legs, const RobotModel & model)
: legs_(legs), model_(model)
{
}

Eigen::Matrix<double, 12, 1> StandController::torques(
  const Eigen::Matrix<double, 12, 1> & q, const Eigen::Matrix<double, 12, 1> & qd,
  const std::array<bool, 4> & contactsUrdf, const Eigen::Matrix3d & R, bool assumeAllFeet) const
{
  std::array<bool, 4> support = contactsUrdf;
  int supportCount = static_cast<int>(std::count(support.begin(), support.end(), true));
  if (supportCount == 0 && assumeAllFeet) {
    support.fill(true);
    supportCount = 4;
  }
  const Eigen::Vector3d gTrunk = R.transpose() * Eigen::Vector3d(0.0, 0.0, -cfg::kG);
  const Eigen::Vector3d fTrunk = supportCount > 0 ?
    Eigen::Vector3d(R.transpose() * Eigen::Vector3d(0.0, 0.0, model_.totalMass() * cfg::kG / supportCount)) :
    Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 12, 1> tau;
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d qi = q.segment<3>(3 * i);
    Eigen::Matrix3d J;
    legs_[i].forwardKinematics(qi, &J);
    Eigen::Vector3d t = legs_[i].gravityCompensation(qi, gTrunk);
    if (support[i]) {t -= J.transpose() * fTrunk;}
    tau.segment<3>(3 * i) = t.cwiseMax(-cfg::kTauMax).cwiseMin(cfg::kTauMax);
  }
  // Joint PD, same gains as the gazebo_ros2_control position PID.
  tau += cfg::kStandKp * (qTarget_ - q) - cfg::kStandKd * qd;
  return tau.cwiseMax(-cfg::kTauLimit).cwiseMin(cfg::kTauLimit);
}

}  // namespace quadru_control
