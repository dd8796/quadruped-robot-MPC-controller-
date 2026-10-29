// Adapted from MIT Cheetah-Software (c71c5a1), user/MIT_Controller/Controllers/convexMPC/
// ConvexMPCLocomotion.h, MIT license.
// QUADRU: plain I/O structs (robot_interface.h), robot parameters instead of MINI_CHEETAH, no
// jump, sparse MPC or remote control; use_wbc = false path (MPC forces + Cartesian leg PD).
#ifndef CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
#define CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H

#include "FootSwingTrajectory.h"
#include "Gait.h"
#include "cppTypes.h"
#include "robot_interface.h"

namespace cheetah
{

class ConvexMPCLocomotion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConvexMPCLocomotion(float dt, int iterationsBetweenMPC, const RobotParams & robot);
  void initialize();

  // One control step. contactPhase: planned stance phase for the estimator (0 in swing).
  void run(
    const StateEstimate<float> & se, const LegData legs[4], const LocomotionCommand & cmd,
    LegCommand commands[4], Vec4<float> & contactPhase);

  // QUADRU: perception. adapter == nullptr: MIT nominal footholds. groundHeight: ground under
  // the body (MIT world), followed by the body height reference.
  void setFootholdAdapter(FootholdAdapter * adapter) {_footholdAdapter = adapter;}
  void setGroundHeight(float h) {_groundHeight = h;}
  // QUADRU: pitch reference from the terrain slope (rad, negative = nose up). MIT uses 0.
  void setTerrainPitch(float pitch) {_terrainPitch = pitch;}

  // Diagnostics: MPC ground reaction forces (world) and last QP time.
  Vec3<float> Fr_des[4];
  float lastSolveMs = 0.f;
  bool lastSolveOk = false;
  // QUADRU: diagnostics. First-step MPC reference of the last solve, MPC state order
  // (roll, pitch, yaw, x, y, z, wx, wy, wz, vx, vy, vz; world), and the MPC period (s).
  const float * reference() const {return trajAll;}
  float mpcPeriod() const {return dtMPC;}

private:
  void setupCommand(const StateEstimate<float> & se, const LocomotionCommand & cmd);
  void updateMPCIfNeeded(int * mpcTable, const StateEstimate<float> & se);
  void solveDenseMPC(int * mpcTable, const StateEstimate<float> & se);

  RobotParams _robot;
  FootholdAdapter * _footholdAdapter = nullptr;
  float _groundHeight = 0.f;
  float _terrainPitch = 0.f;
  float _yaw_turn_rate = 0.f;
  float _yaw_des = 0.f;
  bool _yawHold = false;      // QUADRU: heading hold
  float _yawHoldValue = 0.f;
  float _roll_des = 0.f;
  float _pitch_des = 0.f;
  float _x_vel_des = 0.f;
  float _y_vel_des = 0.f;
  float _body_height;

  int iterationsBetweenMPC;
  int horizonLength;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  OffsetDurationGait trotting, standing;
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  bool firstSwing[4];
  float swingTimeRemaining[4];
  float stand_traj[6];
  int current_gait = 0;

  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  float x_comp_integral = 0;
  Vec3<float> pFoot[4];
  float trajAll[12 * 36] = {};   // QUADRU: zero until the first solve (read by reference())
};

}  // namespace cheetah

#endif  // CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
