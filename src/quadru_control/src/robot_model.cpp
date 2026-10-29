#include "quadru_control/robot_model.hpp"

RobotModel::RobotModel() {
    mass_ = 14.5824;
    totalMass_ = mass_;
    inertia_ << 0.413118,  0.0,       -0.097654,
                0.0,       1.580249,   0.000001,
               -0.097654,  0.000001,   1.549503;
    nominal_feet_ <<  0.297871,  0.297871, -0.286129, -0.286129,
                       0.129903, -0.129903, 0.129903, -0.129903,
                      -0.273,    -0.273,    -0.273,    -0.273;   // feet below whole-body CoM
}

RobotModel RobotModel::trunkOnly() {
    RobotModel m;
    m.mass_ = 5.00095;              // trunk link mass
    m.totalMass_ = 14.5824;         // trunk + 4 legs
    m.inertia_ << 0.0684615,   1.1544e-07,  0.00117595,
                  1.1544e-07,  0.178315,    6.10421e-07,
                  0.00117595,  6.10421e-07, 0.212942;
    m.nominal_feet_.row(2).setConstant(-0.327);   // feet below trunk CoM at home pose
    return m;
}

void RobotModel::compositeInertia(double trunkMass, const Eigen::Vector3d& trunkCom,
                                  const Eigen::Matrix3d& trunkInertia,
                                  const std::array<LegKinematics, 4>& legs,
                                  const Eigen::Matrix<double, 12, 1>& qUrdf,
                                  Eigen::Vector3d& comOut, Eigen::Matrix3d& inertiaOut) {
    // Pass 1: mass, CoM and inertia of every body, trunk frame.
    double totalMass = trunkMass;
    Eigen::Vector3d comNum = trunkMass * trunkCom;
    std::array<LinkPose, 12> poses;   // 3 per leg
    for (int i = 0; i < 4; ++i) {
        auto lp = legs[i].linkPoses(qUrdf.segment<3>(3 * i));
        for (int k = 0; k < 3; ++k) {
            poses[3 * i + k] = lp[k];
            totalMass += lp[k].mass;
            comNum += lp[k].mass * lp[k].com;
        }
    }
    comOut = comNum / totalMass;

    // Pass 2: parallel axis theorem about the system CoM.
    inertiaOut.setZero();
    {
        Eigen::Vector3d d = trunkCom - comOut;
        inertiaOut += trunkInertia + trunkMass * (d.squaredNorm() * Eigen::Matrix3d::Identity() - d * d.transpose());
    }
    for (int j = 0; j < 12; ++j) {
        Eigen::Vector3d d = poses[j].com - comOut;
        inertiaOut += poses[j].inertia + poses[j].mass * (d.squaredNorm() * Eigen::Matrix3d::Identity() - d * d.transpose());
    }

    // Rotate into the MPC body frame (Rf = diag(-1, -1, 1)).
    static const Eigen::Matrix3d Rf = (Eigen::Matrix3d() << -1,0,0, 0,-1,0, 0,0,1).finished();
    comOut = Rf * comOut;
    inertiaOut = Rf * inertiaOut * Rf.transpose();
}
