#pragma once
#include <array>
#include <Eigen/Dense>
#include "quadru_control/leg_kinematics.hpp"

// Mass properties of the robot, from the URDF.
class RobotModel {
public:
    RobotModel();

    // Trunk link only (mass, inertia in the MPC frame); totalMass() is the whole robot.
    static RobotModel trunkOnly();

    // Trunk link CoM in its own frame.
    static Eigen::Vector3d trunkCom() { return {-0.0520205, 0.00128595, 0.509659}; }

    // Composite CoM and inertia of trunk + legs at joint angles qUrdf, about the system CoM.
    // Inputs in the trunk frame; outputs in the MPC frame (Rf = diag(-1, -1, 1)).
    static void compositeInertia(double trunkMass, const Eigen::Vector3d& trunkCom,
                                  const Eigen::Matrix3d& trunkInertia,
                                  const std::array<LegKinematics, 4>& legs,
                                  const Eigen::Matrix<double, 12, 1>& qUrdf,
                                  Eigen::Vector3d& comOut, Eigen::Matrix3d& inertiaOut);

    double mass() const { return mass_; }                 // SRBM mass
    double totalMass() const { return totalMass_; }       // whole robot
    double gravityAccel() const { return 9.81 * totalMass_ / mass_; }   // effective g, trunk-only model
    const Eigen::Matrix3d& inertia() const { return inertia_; }
    // Nominal feet relative to the CoM, MPC frame.
    const Eigen::Matrix<double, 3, 4>& nominalFeet() const { return nominal_feet_; }

private:
    double mass_;
    double totalMass_;
    Eigen::Matrix3d inertia_;
    Eigen::Matrix<double, 3, 4> nominal_feet_;
};
