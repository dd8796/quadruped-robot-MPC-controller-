#pragma once
#include <array>
#include <Eigen/Dense>

// Kinematics of one leg (abduction, flex, knee) from the URDF origins.
struct JointFrame {
    Eigen::Vector3d xyz;   // origin, parent frame
    Eigen::Vector3d rpy;   // orientation, parent frame
};

// Link inertial parameters (URDF <inertial>), in the link frame.
struct LinkInertial {
    double mass = 0.0;
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
};

// Link CoM and inertia about its CoM, both in the trunk frame.
struct LinkPose {
    double mass = 0.0;
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
};

class LegKinematics {
public:
    // footOffset: contact sphere center in the last link frame.
    LegKinematics(const JointFrame& abduction, const JointFrame& flex, const JointFrame& knee,
                  const Eigen::Vector3d& footOffset,
                  const std::array<LinkInertial, 3>& links = {});

    // Foot position and Jacobian (trunk frame).
    Eigen::Vector3d forwardKinematics(const Eigen::Vector3d& q,
                                       Eigen::Matrix3d* jacobian = nullptr) const;

    // Joint axes (trunk frame): foot link angular velocity relative to the trunk = J_w * qd.
    Eigen::Matrix3d angularJacobian(const Eigen::Vector3d& q) const;

    // Hip, flex axis, knee axis, foot (trunk frame), for leg collision checks.
    std::array<Eigen::Vector3d, 4> chainPoints(const Eigen::Vector3d& q) const;

    // Damped least-squares IK (trunk frame); qInit selects the knee branch. Returns true if the
    // error is below tol; qOut holds the last iterate.
    bool inverseKinematics(const Eigen::Vector3d& footTarget, const Eigen::Vector3d& qInit,
                           Eigen::Vector3d& qOut, double tol = 2e-3, int maxIter = 60) const;

    // Gravity compensation torque (trunk fixed); g_body in the trunk frame.
    Eigen::Vector3d gravityCompensation(const Eigen::Vector3d& q, const Eigen::Vector3d& g_body) const;

    // Link masses, CoMs and inertias at q, trunk frame.
    std::array<LinkPose, 3> linkPoses(const Eigen::Vector3d& q) const;

    // Raw URDF data for FloatingBaseModel.
    std::array<JointFrame, 3> jointFrames() const { return {abd_, flex_, knee_}; }
    const std::array<LinkInertial, 3>& linkInertials() const { return links_; }
    const Eigen::Vector3d& footOffset() const { return footOffset_; }

private:
    JointFrame abd_, flex_, knee_;
    std::array<LinkInertial, 3> links_;
    Eigen::Vector3d footOffset_;
};

std::array<LegKinematics, 4> buildAllLegs();
