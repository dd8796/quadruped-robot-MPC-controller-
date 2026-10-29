// Interface between the MIT controller and this robot; replaces ControlFSMData, LegController
// and StateEstimatorContainer (LCM, MIT simulator and Mini Cheetah SPI board).
//
// Cheetah-Software conventions: body frame x forward, y left, z up, origin at the CoM; legs
// 0 FR, 1 FL, 2 HR, 3 HL; rBody: world -> body; quaternions (w, x, y, z).
#ifndef CHEETAH_LOCOMOTION_ROBOT_INTERFACE_H
#define CHEETAH_LOCOMOTION_ROBOT_INTERFACE_H

#include <array>

#include "cppTypes.h"

namespace cheetah
{

// StateEstimate<T> of Cheetah-Software, without LCM.
template<typename T>
struct StateEstimate
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec4<T> contactEstimate = Vec4<T>::Zero();
  Vec3<T> position = Vec3<T>::Zero();
  Vec3<T> vBody = Vec3<T>::Zero();
  Quat<T> orientation = Quat<T>(1, 0, 0, 0);
  Vec3<T> omegaBody = Vec3<T>::Zero();
  RotMat<T> rBody = RotMat<T>::Identity();
  Vec3<T> rpy = Vec3<T>::Zero();
  Vec3<T> omegaWorld = Vec3<T>::Zero();
  Vec3<T> vWorld = Vec3<T>::Zero();
  Vec3<T> aBody = Vec3<T>::Zero();
  Vec3<T> aWorld = Vec3<T>::Zero();
};

// LegControllerData: foot position/velocity relative to the hip, and Jacobian, body frame.
struct LegData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec3<float> p = Vec3<float>::Zero();
  Vec3<float> v = Vec3<float>::Zero();
  Mat3<float> J = Mat3<float>::Zero();
};

// Leg command (LegControllerCommand); LegController::updateCommand applies
// tau = J^T (forceFeedForward + kpCartesian (pDes - p) + kdCartesian (vDes - v)) - kdJoint * qd
struct LegCommand
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec3<float> pDes = Vec3<float>::Zero();
  Vec3<float> vDes = Vec3<float>::Zero();
  Vec3<float> forceFeedForward = Vec3<float>::Zero();
  // QUADRU: swing foot acceleration setpoint (body frame), for leg inertia feedforward.
  Vec3<float> aDes = Vec3<float>::Zero();
  Mat3<float> kpCartesian = Mat3<float>::Zero();
  Mat3<float> kdCartesian = Mat3<float>::Zero();
  Mat3<float> kdJoint = Mat3<float>::Zero();
  void zero() {*this = LegCommand{};}
};

// Robot parameters (replace Quadruped<T> and the Mini Cheetah constants).
struct RobotParams
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  float mass = 9.f;                                   // total mass (single rigid body)
  Mat3<float> inertia = Mat3<float>::Identity();      // composite inertia about the CoM
  std::array<Vec3<float>, 4> hipLocation{};           // hip (abduction axis), body frame
  // Nominal foothold under the hip (body frame, z ignored).
  std::array<Vec3<float>, 4> nominalFoot{};
  float bodyHeight = 0.29f;   // CoM height above the foot plane
  float fMax = 120.f;         // N, max normal force per foot
  float mu = 0.4f;            // friction coefficient
  float swingHeight = 0.06f;  // m
  // Gait (MIT OffsetDurationGait): gaitSegments MPC steps per cycle (= horizon, <= 19), offset
  // and stance duration per leg in steps. Mini Cheetah: 10, (0, 5, 5, 0), 5.
  int gaitSegments = 10;
  Vec4<int> gaitOffsets = Vec4<int>(0, 5, 5, 0);
  Vec4<int> gaitDurations = Vec4<int>(5, 5, 5, 5);
};

// QUADRU: perception foothold adaptation (arXiv 2307.14775), called every tick for each leg
// after the MIT nominal foothold Pf (MIT world frame). swingPhase: 0 in stance, (0, 1] in swing.
// Implementations may modify Pf.
class FootholdAdapter
{
public:
  virtual ~FootholdAdapter() = default;
  virtual void adapt(
    int leg, float swingPhase, float swingTime, const Vec3<float> & pFoot, Vec3<float> & Pf) = 0;
};

// Velocity command, body frame (replaces the remote control).
struct LocomotionCommand
{
  float vx = 0.f;
  float vy = 0.f;
  float yawRate = 0.f;
  bool stand = false;   // MIT gait 4
};

}  // namespace cheetah

#endif  // CHEETAH_LOCOMOTION_ROBOT_INTERFACE_H
