#pragma once
#include <Eigen/Dense>

// Spatial vector algebra (Featherstone 2008). Vectors are [angular; linear] about the frame
// origin.
namespace spatial {

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<     0, -v.z(),  v.y(),
          v.z(),      0, -v.x(),
         -v.y(),  v.x(),      0;
    return S;
}

// Motion transform A -> B; E rotates A into B, r is B's origin in A.
inline Eigen::Matrix<double,6,6> xlateRotate(const Eigen::Matrix3d& E, const Eigen::Vector3d& r) {
    Eigen::Matrix<double,6,6> X = Eigen::Matrix<double,6,6>::Zero();
    X.block<3,3>(0,0) = E;
    X.block<3,3>(3,3) = E;
    X.block<3,3>(3,0) = -E * skew(r);
    return X;
}

// Force transform A -> B (inverse transpose of the motion transform).
inline Eigen::Matrix<double,6,6> forceTransform(const Eigen::Matrix3d& E, const Eigen::Vector3d& r) {
    Eigen::Matrix<double,6,6> X = Eigen::Matrix<double,6,6>::Zero();
    X.block<3,3>(0,0) = E;
    X.block<3,3>(0,3) = -E * skew(r);
    X.block<3,3>(3,3) = E;
    return X;
}

// Spatial cross product, motion vectors.
inline Eigen::Matrix<double,6,6> crossMotion(const Eigen::Matrix<double,6,1>& v) {
    Eigen::Matrix<double,6,6> C = Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix3d Sw = skew(v.head<3>()), Sv = skew(v.tail<3>());
    C.block<3,3>(0,0) = Sw;
    C.block<3,3>(3,0) = Sv;
    C.block<3,3>(3,3) = Sw;
    return C;
}

// Spatial cross product, force vectors.
inline Eigen::Matrix<double,6,6> crossForce(const Eigen::Matrix<double,6,1>& v) {
    return -crossMotion(v).transpose();
}

// Spatial inertia about the frame origin; c: CoM, Ic: inertia about the CoM.
inline Eigen::Matrix<double,6,6> spatialInertia(double m, const Eigen::Vector3d& c, const Eigen::Matrix3d& Ic) {
    Eigen::Matrix<double,6,6> I = Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix3d Sc = skew(c);
    I.block<3,3>(0,0) = Ic + m * Sc * Sc.transpose();
    I.block<3,3>(0,3) = m * Sc;
    I.block<3,3>(3,0) = m * Sc.transpose();
    I.block<3,3>(3,3) = m * Eigen::Matrix3d::Identity();
    return I;
}

}  // namespace spatial
