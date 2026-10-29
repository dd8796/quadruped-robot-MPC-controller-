// Adapted from MIT Cheetah-Software (c71c5a1): OrientationEstimator, LinearKFPositionVelocity-
// Estimator and ContactEstimator, MIT license.
// QUADRU: single class without LCM; noise from mini-cheetah-defaults.yaml; initial yaw kept
// (world frame = map frame); initialized at the start pose. Contact = planned gait phase.
#ifndef CHEETAH_LOCOMOTION_STATE_ESTIMATOR_H
#define CHEETAH_LOCOMOTION_STATE_ESTIMATOR_H

#include "cppTypes.h"
#include "robot_interface.h"

namespace cheetah
{

class StateEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Params
  {
    float dt = 0.001f;
    float imuProcessNoisePosition = 0.02f;
    float imuProcessNoiseVelocity = 0.02f;
    float footProcessNoisePosition = 0.002f;
    float footSensorNoisePosition = 0.001f;
    float footSensorNoiseVelocity = 0.1f;
    float footHeightSensorNoise = 0.001f;
  };

  StateEstimator(const RobotParams & robot, const Params & params);

  // Start pose: body at (0, 0, bodyHeight), feet at their measured positions.
  void reset(const Quat<float> & orientation, const LegData legs[4]);

  // imuOrientation: body -> world; gyro, acc: body frame (specific force).
  void run(
    const Quat<float> & imuOrientation, const Vec3<float> & gyro, const Vec3<float> & acc,
    const LegData legs[4], const Vec4<float> & contactPhase);
  // QUADRU: ground height under each foot (MIT world). MIT assumes 0.
  void setFootGroundHeights(const Vec4<float> & h) {_footGround = h;}
  // QUADRU: external localization (MIT FSM_State_Vision): pose replaced, velocities kept.
  void overridePose(const Vec3<float> & position, const Quat<float> & orientation);

  const StateEstimate<float> & result() const {return _result;}

private:
  void runOrientation(const Quat<float> & q, const Vec3<float> & gyro, const Vec3<float> & acc);
  void runPositionVelocity(const LegData legs[4]);

  RobotParams _robot;
  Params _params;
  StateEstimate<float> _result;
  Vec4<float> _footGround = Vec4<float>::Zero();

  Eigen::Matrix<float, 18, 1> _xhat;
  Eigen::Matrix<float, 12, 1> _ps;
  Eigen::Matrix<float, 12, 1> _vs;
  Eigen::Matrix<float, 18, 18> _A;
  Eigen::Matrix<float, 18, 18> _Q0;
  Eigen::Matrix<float, 18, 18> _P;
  Eigen::Matrix<float, 28, 28> _R0;
  Eigen::Matrix<float, 18, 3> _B;
  Eigen::Matrix<float, 28, 18> _C;
};

}  // namespace cheetah

#endif  // CHEETAH_LOCOMOTION_STATE_ESTIMATOR_H
