// Adapted from MIT Cheetah-Software (c71c5a1), user/MIT_Controller/Controllers/convexMPC/
// ConvexMPCLocomotion.cpp, MIT license. Changes are marked QUADRU.
#include "ConvexMPCLocomotion.h"

#include <chrono>
#include <cmath>
#include <cstdio>

#include "Math/orientation_tools.h"
#include "RobotState.h"
#include "convexMPC_interface.h"

namespace cheetah
{

namespace
{
// MIT default user parameters (config/mc-mit-ctrl-user-parameters.yaml).
constexpr float kCmpcXDrag = 3.f;
constexpr float kCmpcBonusSwing = 0.f;
}  // namespace

ConvexMPCLocomotion::ConvexMPCLocomotion(
  float _dt, int _iterations_between_mpc, const RobotParams & robot)
: _robot(robot),
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(robot.gaitSegments),   // QUADRU: robot gait (MIT: 10)
  dt(_dt),
  trotting(horizonLength, robot.gaitOffsets, robot.gaitDurations, "Trotting"),
  standing(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>::Constant(horizonLength), "Standing")
{
  dtMPC = dt * iterationsBetweenMPC;
  _body_height = _robot.bodyHeight;
  std::printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  // QUADRU: robot mass/inertia, friction and max force.
  ::RobotState::setBodyParameters(_robot.mass, _robot.inertia);
  setup_problem(dtMPC, horizonLength, _robot.mu, _robot.fMax);
  rpy_comp.setZero();
  rpy_int.setZero();
  for (int i = 0; i < 4; i++) {
    firstSwing[i] = true;
    f_ff[i].setZero();
    Fr_des[i].setZero();
  }
  world_position_desired.setZero();
}

void ConvexMPCLocomotion::initialize()
{
  for (int i = 0; i < 4; i++) {firstSwing[i] = true;}
  firstRun = true;
  iterationCounter = 0;
  _yawHold = false;
}

void ConvexMPCLocomotion::setupCommand(const StateEstimate<float> & se, const LocomotionCommand & cmd)
{
  // QUADRU: robot body height above the terrain; command from LocomotionCommand.
  _body_height = _robot.bodyHeight + _groundHeight;
  const float filter = 0.1f;
  _yaw_turn_rate = cmd.yawRate;
  _x_vel_des = _x_vel_des * (1 - filter) + cmd.vx * filter;
  _y_vel_des = _y_vel_des * (1 - filter) + cmd.vy * filter;
  // QUADRU: heading hold. With zero yaw rate the heading is frozen and tracked (MIT drifts
  // ~12 deg / 10 m); the setpoint is wrapped to within pi of the measured yaw.
  if (std::fabs(_yaw_turn_rate) < 1e-3f) {
    if (!_yawHold) {
      _yawHold = true;
      _yawHoldValue = se.rpy[2];
    }
    _yaw_des = se.rpy[2] + std::remainder(_yawHoldValue - se.rpy[2], 2.f * float(M_PI));
  } else {
    _yawHold = false;
    _yaw_des = se.rpy[2] + dt * _yaw_turn_rate;
  }
  _roll_des = 0.;
  _pitch_des = _terrainPitch;   // QUADRU: terrain slope (MIT: 0)
}

void ConvexMPCLocomotion::run(
  const StateEstimate<float> & seResult, const LegData legs[4], const LocomotionCommand & cmd,
  LegCommand commands[4], Vec4<float> & contactPhase)
{
  setupCommand(seResult, cmd);
  const int gaitNumber = cmd.stand ? 4 : 0;

  // Check if transition to standing
  if (((gaitNumber == 4) && current_gait != 4) || firstRun) {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = 0.21;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  Gait * gait = &trotting;
  if (gaitNumber == 4) {gait = &standing;}
  current_gait = gaitNumber;
  gait->setIterations(iterationsBetweenMPC, iterationCounter);

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  // Integral-esque pitch and roll compensation
  if (std::fabs(v_robot[0]) > .2) {
    rpy_int[1] += dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
  }
  if (std::fabs(v_robot[1]) > 0.1) {
    rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];
  }
  rpy_int[0] = std::fmin(std::fmax(rpy_int[0], -.25f), .25f);
  rpy_int[1] = std::fmin(std::fmax(rpy_int[1], -.25f), .25f);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0];

  for (int i = 0; i < 4; i++) {
    pFoot[i] = seResult.position + seResult.rBody.transpose() * (_robot.hipLocation[i] + legs[i].p);
  }

  if (gait != &standing) {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if (firstRun) {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];
    for (int i = 0; i < 4; i++) {
      footSwingTrajectories[i].setHeight(_robot.swingHeight);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
    }
    firstRun = false;
  }

  // foot placement
  for (int l = 0; l < 4; l++) {swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);}

  float side_sign[4] = {-1, 1, -1, 1};
  (void)side_sign;
  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  float interleave_gain = -0.2;
  float v_abs = std::fabs(v_des_robot[0]);
  for (int i = 0; i < 4; i++) {
    if (firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
    footSwingTrajectories[i].setHeight(_robot.swingHeight);
    // QUADRU: robot nominal foothold (MIT: Mini Cheetah hip offset).
    Vec3<float> pRobotFrame(_robot.nominalFoot[i][0], _robot.nominalFoot[i][1], 0.f);

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);
    Vec3<float> pYawCorrected =
      ori::coordinateRotation(ori::CoordinateAxis::Z, -_yaw_turn_rate * stance_time / 2) * pRobotFrame;

    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() *
      (pYawCorrected + des_vel * swingTimeRemaining[i]);

    float p_rel_max = 0.3f;
    // Using the estimated velocity is correct
    float pfx_rel = seResult.vWorld[0] * (.5 + kCmpcBonusSwing) * stance_time +
      .03f * (seResult.vWorld[0] - v_des_world[0]) +
      (0.5f * seResult.position[2] / 9.81f) * (seResult.vWorld[1] * _yaw_turn_rate);
    float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +
      .03f * (seResult.vWorld[1] - v_des_world[1]) +
      (0.5f * seResult.position[2] / 9.81f) * (-seResult.vWorld[0] * _yaw_turn_rate);
    pfx_rel = std::fmin(std::fmax(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = std::fmin(std::fmax(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel;
    Pf[2] = -0.003;
    // QUADRU: visual foothold adaptation.
    if (_footholdAdapter) {
      const float swingPhase = gait->getSwingState()[i];
      _footholdAdapter->adapt(i, swingPhase, swingTimes[i], pFoot[i], Pf);
    }
    footSwingTrajectories[i].setFinalPosition(Pf);
  }

  // calc gait
  iterationCounter++;

  // load LCM leg swing gains
  Kp << 700, 0, 0,
    0, 700, 0,
    0, 0, 150;
  Kp_stance = 0 * Kp;
  Kd << 7, 0, 0,
    0, 7, 0,
    0, 0, 7;
  Kd_stance = Kd;

  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int * mpcTable = gait->getMpcTable();
  updateMPCIfNeeded(mpcTable, seResult);

  Vec4<float> se_contactState(0, 0, 0, 0);
  for (int foot = 0; foot < 4; foot++) {
    commands[foot].zero();   // QUADRU: LegController::zeroCommand()
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    if (swingState > 0) {   // foot is in swing
      if (firstSwing[foot]) {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - _robot.hipLocation[foot];
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      commands[foot].pDes = pDesLeg;
      commands[foot].vDes = vDesLeg;
      commands[foot].kpCartesian = Kp;
      commands[foot].kdCartesian = Kd;
      commands[foot].aDes = seResult.rBody * footSwingTrajectories[foot].getAcceleration();   // QUADRU
    } else {   // foot is in stance
      firstSwing[foot] = true;
      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - _robot.hipLocation[foot];
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      commands[foot].pDes = pDesLeg;
      commands[foot].vDes = vDesLeg;
      commands[foot].kpCartesian = Kp_stance;
      commands[foot].kdCartesian = Kd_stance;
      commands[foot].forceFeedForward = f_ff[foot];
      commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;
      se_contactState[foot] = contactState;
    }
  }
  contactPhase = se_contactState;   // data._stateEstimator->setContactPhase(se_contactState)
}

void ConvexMPCLocomotion::updateMPCIfNeeded(int * mpcTable, const StateEstimate<float> & seResult)
{
  if ((iterationCounter % iterationsBetweenMPC) == 0) {
    const float * p = seResult.position.data();
    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
    Vec3<float> v_des_world = seResult.rBody.transpose() * v_des_robot;

    if (current_gait == 4) {
      float trajInitial[12] = {
        _roll_des, _pitch_des, (float)stand_traj[5], (float)stand_traj[0], (float)stand_traj[1],
        (float)_body_height, 0, 0, 0, 0, 0, 0};
      for (int i = 0; i < horizonLength; i++) {
        for (int j = 0; j < 12; j++) {trajAll[12 * i + j] = trajInitial[j];}
      }
    } else {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];
      if (xStart - p[0] > max_pos_error) {xStart = p[0] + max_pos_error;}
      if (p[0] - xStart > max_pos_error) {xStart = p[0] - max_pos_error;}
      if (yStart - p[1] > max_pos_error) {yStart = p[1] + max_pos_error;}
      if (p[1] - yStart > max_pos_error) {yStart = p[1] - max_pos_error;}
      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      float trajInitial[12] = {
        (float)rpy_comp[0], (float)rpy_comp[1] + _pitch_des, _yaw_des, xStart, yStart, (float)_body_height,
        0, 0, _yaw_turn_rate, v_des_world[0], v_des_world[1], 0};
      for (int i = 0; i < horizonLength; i++) {
        for (int j = 0; j < 12; j++) {trajAll[12 * i + j] = trajInitial[j];}
        if (i == 0) {
          trajAll[2] = _yawHold ? _yaw_des : seResult.rpy[2];   // QUADRU: heading hold
        } else {
          trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }
    solveDenseMPC(mpcTable, seResult);
  }
}

void ConvexMPCLocomotion::solveDenseMPC(int * mpcTable, const StateEstimate<float> & seResult)
{
  float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};
  float yaw = seResult.rpy[2];
  float * weights = Q;
  // QUADRU: scale the force regularization with mass (alpha * m^2 constant; MIT: 4e-5 at 9 kg).
  float alpha = 4e-5f * (9.f / _robot.mass) * (9.f / _robot.mass);
  float * p = const_cast<float *>(seResult.position.data());
  float * v = const_cast<float *>(seResult.vWorld.data());
  float * w = const_cast<float *>(seResult.omegaWorld.data());
  float * q = const_cast<float *>(seResult.orientation.data());

  float r[12];
  for (int i = 0; i < 12; i++) {r[i] = pFoot[i % 4][i / 4] - seResult.position[i / 4];}

  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  const auto t0 = std::chrono::steady_clock::now();
  dtMPC = dt * iterationsBetweenMPC;
  setup_problem(dtMPC, horizonLength, _robot.mu, _robot.fMax);
  update_x_drag(x_comp_integral);
  if (vxy[0] > 0.3 || vxy[0] < -0.3) {
    x_comp_integral += kCmpcXDrag * pz_err * dtMPC / vxy[0];
  }
  update_solver_settings(10000, 1e-7, 1e-8, 1.5, 0.1, 0);   // use_jcqp = 0: qpOASES
  update_problem_data_floats(p, v, q, w, r, yaw, weights, trajAll, alpha, mpcTable);
  lastSolveMs = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
  lastSolveOk = get_solve_status() == 0;

  for (int leg = 0; leg < 4; leg++) {
    Vec3<float> f;
    for (int axis = 0; axis < 3; axis++) {f[axis] = get_solution(leg * 3 + axis);}
    f_ff[leg] = -seResult.rBody * f;
    Fr_des[leg] = f;
  }
}

}  // namespace cheetah
