#pragma once
#include <Eigen/Dense>
#include <array>
#include "quadru_control/leg_kinematics.hpp"

// Floating-base rigid-body dynamics M(q) nu_dot + h(q, nu) = tau, Featherstone CRBA/RNEA.
// nu = [omega_base; v_base] (trunk frame) ++ 12 joint rates (URDF order). The base is a free
// joint (S_0 = I_6).
class FloatingBaseModel {
public:
    static constexpr int kNb = 13;   // 0 = trunk, 1..12 = leg links
    static constexpr int kNv = 18;

    FloatingBaseModel(double trunkMass, const Eigen::Vector3d& trunkCom,
                       const Eigen::Matrix3d& trunkInertia,
                       const std::array<LegKinematics, 4>& legs);

    // Mass matrix (18x18), depends on joint angles only.
    Eigen::Matrix<double, kNv, kNv> massMatrix(const Eigen::Matrix<double, 12, 1>& qJoints) const;

    // h = C(q, v) v + g(q): RNEA with zero acceleration. R0: trunk -> world.
    Eigen::Matrix<double, kNv, 1> biasForces(const Eigen::Matrix3d& R0,
                                              const Eigen::Matrix<double, 12, 1>& qJoints,
                                              const Eigen::Matrix<double, 6, 1>& vBase,
                                              const Eigen::Matrix<double, 12, 1>& vJoints) const;

    // RNEA. Also used to cross-check massMatrix() column by column.
    Eigen::Matrix<double, kNv, 1> inverseDynamics(const Eigen::Matrix3d& R0,
                                                   const Eigen::Matrix<double, 12, 1>& qJoints,
                                                   const Eigen::Matrix<double, 6, 1>& vBase,
                                                   const Eigen::Matrix<double, 12, 1>& vJoints,
                                                   const Eigen::Matrix<double, 6, 1>& aBase,
                                                   const Eigen::Matrix<double, 12, 1>& aJoints) const;

    // d(foot velocity, trunk frame) / d(nu).
    Eigen::Matrix<double, 3, kNv> contactJacobian(const Eigen::Matrix<double, 12, 1>& qJoints,
                                                   const std::array<LegKinematics, 4>& legs, int leg) const;

    // Foot acceleration at zero generalized acceleration (J_dot * nu), trunk frame.
    std::array<Eigen::Vector3d, 4> footBiasAcceleration(const Eigen::Matrix<double, 12, 1>& qJoints,
                                                          const std::array<LegKinematics, 4>& legs,
                                                          const Eigen::Matrix<double, 6, 1>& vBase,
                                                          const Eigen::Matrix<double, 12, 1>& vJoints) const;

private:
    struct Body {
        int parent;                        // -1 for the trunk
        Eigen::Matrix<double, 6, 6> I;      // about the body frame origin
    };
    std::array<Body, kNb> bodies_;
    std::array<JointFrame, 12> jointFrame_;  // fixed parent -> pre-joint transform

    void computeXup(const Eigen::Matrix<double, 12, 1>& qJoints,
                     std::array<Eigen::Matrix<double, 6, 6>, kNb>& Xup) const;

    // Body frame -> trunk frame rotations (propagateVA works in body frames).
    std::array<Eigen::Matrix3d, kNb> computeRToTrunk(const Eigen::Matrix<double, 12, 1>& qJoints) const;

    // Velocity/acceleration propagation, no gravity or forces.
    void propagateVA(const std::array<Eigen::Matrix<double, 6, 6>, kNb>& Xup,
                      const Eigen::Matrix<double, 6, 1>& vBase, const Eigen::Matrix<double, 12, 1>& vJoints,
                      const Eigen::Matrix<double, 6, 1>& aBase, const Eigen::Matrix<double, 12, 1>& aJoints,
                      std::array<Eigen::Matrix<double, 6, 1>, kNb>& v,
                      std::array<Eigen::Matrix<double, 6, 1>, kNb>& a) const;
};
