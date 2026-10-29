// Adapted from MIT Cheetah-Software (c71c5a1), see StateEstimator.h. Filter equations unchanged.
#include "StateEstimator.h"

#include <cmath>

#include "Math/orientation_tools.h"

namespace cheetah
{

StateEstimator::StateEstimator(const RobotParams & robot, const Params & params)
: _robot(robot), _params(params)
{
  // ---- LinearKFPositionVelocityEstimator<T>::setup()
  const float dt = _params.dt;
  _xhat.setZero();
  _ps.setZero();
  _vs.setZero();
  _A.setZero();
  _A.block(0, 0, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
  _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<float, 3, 3>::Identity();
  _A.block(3, 3, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
  _A.block(6, 6, 12, 12) = Eigen::Matrix<float, 12, 12>::Identity();
  _B.setZero();
  _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<float, 3, 3>::Identity();
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<float, 3, 3>::Identity(), Eigen::Matrix<float, 3, 3>::Zero();
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<float, 3, 3>::Zero(), Eigen::Matrix<float, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  _C.block(6, 0, 3, 6) = C1;
  _C.block(9, 0, 3, 6) = C1;
  _C.block(0, 6, 12, 12) = float(-1) * Eigen::Matrix<float, 12, 12>::Identity();
  _C.block(12, 0, 3, 6) = C2;
  _C.block(15, 0, 3, 6) = C2;
  _C.block(18, 0, 3, 6) = C2;
  _C.block(21, 0, 3, 6) = C2;
  _C(27, 17) = float(1);
  _C(26, 14) = float(1);
  _C(25, 11) = float(1);
  _C(24, 8) = float(1);
  _P.setIdentity();
  _P = float(100) * _P;
  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<float, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3) = (dt * 9.8f / 20.f) * Eigen::Matrix<float, 3, 3>::Identity();
  _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<float, 12, 12>::Identity();
  _R0.setIdentity();
}

void StateEstimator::reset(const Quat<float> & orientation, const LegData legs[4])
{
  // QUADRU: start at the initial pose (MIT: _xhat = 0).
  runOrientation(orientation, Vec3<float>::Zero(), Vec3<float>(0, 0, 9.81f));
  _xhat.setZero();
  _xhat.segment<3>(0) = Vec3<float>(0, 0, _robot.bodyHeight);
  const Mat3<float> Rbod = _result.rBody.transpose();
  for (int i = 0; i < 4; i++) {
    _xhat.segment<3>(6 + 3 * i) = _xhat.segment<3>(0) + Rbod * (_robot.hipLocation[i] + legs[i].p);
  }
  _P.setIdentity();
  _P = float(100) * _P;
  _result.position = _xhat.segment<3>(0);
  _result.vWorld.setZero();
  _result.vBody.setZero();
}

void StateEstimator::run(
  const Quat<float> & imuOrientation, const Vec3<float> & gyro, const Vec3<float> & acc,
  const LegData legs[4], const Vec4<float> & contactPhase)
{
  // MIT StateEstimatorContainer order: orientation, position/velocity, contact.
  runOrientation(imuOrientation, gyro, acc);
  runPositionVelocity(legs);
  _result.contactEstimate = contactPhase;   // ContactEstimator::run()
}

void StateEstimator::overridePose(const Vec3<float> & position, const Quat<float> & orientation)
{
  // FSM_State_Vision<T>::_updateStateEstimator()
  _result.position = position;
  _result.orientation = orientation;
  _result.rpy = ori::quatToRPY(_result.orientation);
  _result.rBody = ori::quaternionToRotationMatrix(_result.orientation);
}

void StateEstimator::runOrientation(const Quat<float> & q, const Vec3<float> & gyro, const Vec3<float> & acc)
{
  // ---- VectorNavOrientationEstimator<T>::run() (initial yaw kept)
  _result.orientation = q;
  _result.rpy = ori::quatToRPY(_result.orientation);
  _result.rBody = ori::quaternionToRotationMatrix(_result.orientation);
  _result.omegaBody = gyro;
  _result.omegaWorld = _result.rBody.transpose() * _result.omegaBody;
  _result.aBody = acc;
  _result.aWorld = _result.rBody.transpose() * _result.aBody;
}

void StateEstimator::runPositionVelocity(const LegData legs[4])
{
  // ---- LinearKFPositionVelocityEstimator<T>::run()
  float process_noise_pimu = _params.imuProcessNoisePosition;
  float process_noise_vimu = _params.imuProcessNoiseVelocity;
  float process_noise_pfoot = _params.footProcessNoisePosition;
  float sensor_noise_pimu_rel_foot = _params.footSensorNoisePosition;
  float sensor_noise_vimu_rel_foot = _params.footSensorNoiseVelocity;
  float sensor_noise_zfoot = _params.footHeightSensorNoise;

  Eigen::Matrix<float, 18, 18> Q = Eigen::Matrix<float, 18, 18>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
  Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
  Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

  Eigen::Matrix<float, 28, 28> R = Eigen::Matrix<float, 28, 28>::Identity();
  R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * sensor_noise_pimu_rel_foot;
  R.block(12, 12, 12, 12) = _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot;
  R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * sensor_noise_zfoot;

  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;

  Vec3<float> g(0, 0, float(-9.81));
  Mat3<float> Rbod = _result.rBody.transpose();
  Vec3<float> a = _result.aWorld + g;   // in old code, Rbod * se_acc + g
  Vec4<float> pzs = Vec4<float>::Zero();
  Vec4<float> trusts = Vec4<float>::Zero();
  Vec3<float> p0, v0;
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];

  for (int i = 0; i < 4; i++) {
    int i1 = 3 * i;
    Vec3<float> ph = _robot.hipLocation[i];   // hip positions relative to CoM
    Vec3<float> p_rel = ph + legs[i].p;
    Vec3<float> dp_rel = legs[i].v;
    Vec3<float> p_f = Rbod * p_rel;
    Vec3<float> dp_f = Rbod * (_result.omegaBody.cross(p_rel) + dp_rel);

    qindex = 6 + i1;
    rindex1 = i1;
    rindex2 = 12 + i1;
    rindex3 = 24 + i;

    float trust = float(1);
    float phase = std::fmin(_result.contactEstimate(i), float(1));
    float trust_window = float(0.2);
    if (phase < trust_window) {
      trust = phase / trust_window;
    } else if (phase > (float(1) - trust_window)) {
      trust = (float(1) - phase) / trust_window;
    }
    float high_suspect_number(100);

    Q.block(qindex, qindex, 3, 3) =
      (float(1) + (float(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
    R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
    R.block(rindex2, rindex2, 3, 3) =
      (float(1) + (float(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
    R(rindex3, rindex3) = (float(1) + (float(1) - trust) * high_suspect_number) * R(rindex3, rindex3);

    trusts(i) = trust;
    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
    // QUADRU: stance foot at the ground height under it (MIT: 0).
    pzs(i) = (1.0f - trust) * (p0(2) + p_f(2)) + trust * _footGround(i);
  }

  Eigen::Matrix<float, 28, 1> y;
  y << _ps, _vs, pzs;
  _xhat = _A * _xhat + _B * a;
  Eigen::Matrix<float, 18, 18> At = _A.transpose();
  Eigen::Matrix<float, 18, 18> Pm = _A * _P * At + Q;
  Eigen::Matrix<float, 18, 28> Ct = _C.transpose();
  Eigen::Matrix<float, 28, 1> yModel = _C * _xhat;
  Eigen::Matrix<float, 28, 1> ey = y - yModel;
  Eigen::Matrix<float, 28, 28> S = _C * Pm * Ct + R;

  Eigen::Matrix<float, 28, 1> S_ey = S.lu().solve(ey);
  _xhat += Pm * Ct * S_ey;

  Eigen::Matrix<float, 28, 18> S_C = S.lu().solve(_C);
  _P = (Eigen::Matrix<float, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;

  Eigen::Matrix<float, 18, 18> Pt = _P.transpose();
  _P = (_P + Pt) / float(2);

  if (_P.block(0, 0, 2, 2).determinant() > float(0.000001)) {
    _P.block(0, 2, 2, 16).setZero();
    _P.block(2, 0, 16, 2).setZero();
    _P.block(0, 0, 2, 2) /= float(10);
  }

  _result.position = _xhat.block(0, 0, 3, 1);
  _result.vWorld = _xhat.block(3, 0, 3, 1);
  _result.vBody = _result.rBody * _result.vWorld;
}

}  // namespace cheetah
