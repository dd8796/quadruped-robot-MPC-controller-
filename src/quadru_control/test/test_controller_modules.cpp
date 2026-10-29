// Unit tests of the controller modules (no ROS, no Gazebo).
#include <gtest/gtest.h>

#include "quadru_control/contact_sensing.hpp"
#include "quadru_control/controller_config.hpp"
#include "quadru_control/stand_controller.hpp"

using namespace quadru_control;

// The same message read again on every tick must not confirm a contact.
TEST(ContactDebouncer, CountsEachMessageOnce)
{
  ContactDebouncer d;
  d.reset();
  ContactSample s;
  s.contact = true;
  s.stampNs = 1000;
  for (int tick = 0; tick < 50; ++tick) {
    EXPECT_FALSE(d.update(0, s, s.stampNs + 1000));
  }
  bool confirmed = false;
  for (int k = 1; k < cfg::kHysteresisConfirm; ++k) {
    s.stampNs += 5000000;
    confirmed = d.update(0, s, s.stampNs);
  }
  EXPECT_TRUE(confirmed);
  // Stale message: not a fresh contact.
  EXPECT_FALSE(ContactDebouncer::isFresh(s, s.stampNs + 2 * cfg::kSensorTimeoutNs));
}

// Stance legs carry the total weight M g; a swing leg only its own gravity.
TEST(StandController, SupportLegsCarryTotalWeightOthersOnlyTheirOwnGravity)
{
  const auto legs = buildAllLegs();
  const RobotModel model = RobotModel::trunkOnly();
  StandController stand(legs, model);
  Eigen::Matrix<double, 12, 1> q;
  for (int i = 0; i < 4; ++i) {q.segment<3>(3 * i) << 0.0, 0.52611, 1.11101567;}
  stand.setTarget(q);   // at target, zero velocity: no PD term
  const std::array<bool, 4> contacts = {true, false, false, true};
  const Eigen::Vector3d gTrunk(0.0, 0.0, -cfg::kG);
  const auto tau = stand.torques(
    q, Eigen::Matrix<double, 12, 1>::Zero(), contacts, Eigen::Matrix3d::Identity(), false);
  ASSERT_LT(tau.cwiseAbs().maxCoeff(), cfg::kTauLimit);   // no saturation
  Eigen::Vector3d fSum = Eigen::Vector3d::Zero();
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d qi = q.segment<3>(3 * i);
    Eigen::Matrix3d J;
    legs[i].forwardKinematics(qi, &J);
    const Eigen::Vector3d g = legs[i].gravityCompensation(qi, gTrunk);
    if (!contacts[i]) {
      EXPECT_LT((tau.segment<3>(3 * i) - g).norm(), 1e-12);
      continue;
    }
    // tau = G - J^T f  ->  f = J^-T (G - tau)
    const Eigen::Vector3d f = J.transpose().fullPivLu().solve(g - tau.segment<3>(3 * i));
    EXPECT_NEAR(f.z(), model.totalMass() * cfg::kG / 2.0, 1e-9);
    fSum += f;
  }
  EXPECT_NEAR(fSum.z(), model.totalMass() * cfg::kG, 1e-9);
  EXPECT_LT(fSum.head<2>().norm(), 1e-9);
}

// The knee -> foot vector is rigid with the foot link: its rate is (J_w qd) x (foot - knee).
TEST(LegKinematics, AngularJacobianMatchesFootLinkRotation)
{
  const auto legs = buildAllLegs();
  const Eigen::Vector3d q(0.1, 0.52611, 1.11101567);
  const Eigen::Vector3d qd(0.7, -1.3, 2.1);
  const double eps = 1e-6;
  for (const auto & leg : legs) {
    const auto p0 = leg.chainPoints(q);
    const auto p1 = leg.chainPoints(q + eps * qd);
    const Eigen::Vector3d rate = ((p1[3] - p1[2]) - (p0[3] - p0[2])) / eps;
    const Eigen::Vector3d w = leg.angularJacobian(q) * qd;
    EXPECT_LT((rate - w.cross(p0[3] - p0[2])).norm(), 1e-5);
  }
}
