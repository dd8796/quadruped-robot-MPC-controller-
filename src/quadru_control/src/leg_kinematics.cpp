#include "quadru_control/leg_kinematics.hpp"
#include <cmath>

static Eigen::Matrix3d rpy2R(const Eigen::Vector3d& r) {
    double cr = std::cos(r(0)), sr = std::sin(r(0));
    double cp = std::cos(r(1)), sp = std::sin(r(1));
    double cy = std::cos(r(2)), sy = std::sin(r(2));
    Eigen::Matrix3d Rz_, Ry_, Rx_;
    Rz_ << cy, -sy, 0,  sy, cy, 0,  0, 0, 1;
    Ry_ << cp, 0, sp,   0, 1, 0,   -sp, 0, cp;
    Rx_ << 1, 0, 0,   0, cr, -sr,   0, sr, cr;
    return Rz_ * Ry_ * Rx_;
}

static Eigen::Matrix4d makeT(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = rpy2R(rpy);
    T.block<3,1>(0,3) = xyz;
    return T;
}

static Eigen::Matrix4d rz(double q) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    double c = std::cos(q), s = std::sin(q);
    T(0,0)=c; T(0,1)=-s; T(1,0)=s; T(1,1)=c;
    return T;
}

LegKinematics::LegKinematics(const JointFrame& abduction, const JointFrame& flex, const JointFrame& knee,
                              const Eigen::Vector3d& footOffset,
                              const std::array<LinkInertial, 3>& links)
    : abd_(abduction), flex_(flex), knee_(knee), links_(links), footOffset_(footOffset) {}

Eigen::Vector3d LegKinematics::gravityCompensation(const Eigen::Vector3d& q,
                                                    const Eigen::Vector3d& g_body) const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Vector3d axes[3], origins[3], coms[3];
    const JointFrame* frames[3] = {&abd_, &flex_, &knee_};
    for (int k = 0; k < 3; ++k) {
        T = T * makeT(frames[k]->xyz, frames[k]->rpy);
        axes[k] = T.block<3,3>(0,0) * Eigen::Vector3d(0,0,1);
        origins[k] = T.block<3,1>(0,3);
        T = T * rz(q(k));
        Eigen::Vector4d c; c << links_[k].com, 1.0;
        coms[k] = (T * c).head<3>();   // link k is rigid with the frame after q_k
    }
    Eigen::Vector3d tau = Eigen::Vector3d::Zero();
    for (int k = 0; k < 3; ++k) {
        double Qk = 0.0;   // generalized gravity force on joint k
        for (int j = k; j < 3; ++j)
            Qk += links_[j].mass * g_body.dot(axes[k].cross(coms[j] - origins[k]));
        tau(k) = -Qk;
    }
    return tau;
}

std::array<LinkPose, 3> LegKinematics::linkPoses(const Eigen::Vector3d& q) const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const JointFrame* frames[3] = {&abd_, &flex_, &knee_};
    std::array<LinkPose, 3> out;
    for (int k = 0; k < 3; ++k) {
        T = T * makeT(frames[k]->xyz, frames[k]->rpy) * rz(q(k));
        Eigen::Matrix3d R = T.block<3,3>(0,0);
        Eigen::Vector4d c; c << links_[k].com, 1.0;
        out[k].mass = links_[k].mass;
        out[k].com = (T * c).head<3>();
        out[k].inertia = R * links_[k].inertia * R.transpose();
    }
    return out;
}

Eigen::Vector3d LegKinematics::forwardKinematics(const Eigen::Vector3d& q, Eigen::Matrix3d* jacobian) const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double,3,3> axes, origins;
    const JointFrame* frames[3] = {&abd_, &flex_, &knee_};

    for (int k = 0; k < 3; ++k) {
        T = T * makeT(frames[k]->xyz, frames[k]->rpy);
        axes.col(k) = T.block<3,3>(0,0) * Eigen::Vector3d(0,0,1);
        origins.col(k) = T.block<3,1>(0,3);
        T = T * rz(q(k));
    }
    Eigen::Vector4d footH; footH << footOffset_, 1.0;
    Eigen::Vector3d pFoot = (T * footH).head<3>();

    if (jacobian) {
        for (int k = 0; k < 3; ++k)
            jacobian->col(k) = axes.col(k).cross(pFoot - origins.col(k));
    }
    return pFoot;
}

Eigen::Matrix3d LegKinematics::angularJacobian(const Eigen::Vector3d& q) const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const JointFrame* frames[3] = {&abd_, &flex_, &knee_};
    Eigen::Matrix3d Jw;
    for (int k = 0; k < 3; ++k) {
        T = T * makeT(frames[k]->xyz, frames[k]->rpy);
        Jw.col(k) = T.block<3,3>(0,0) * Eigen::Vector3d(0,0,1);
        T = T * rz(q(k));
    }
    return Jw;
}

std::array<Eigen::Vector3d, 4> LegKinematics::chainPoints(const Eigen::Vector3d& q) const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const JointFrame* frames[3] = {&abd_, &flex_, &knee_};
    std::array<Eigen::Vector3d, 4> pts;
    for (int k = 0; k < 3; ++k) {
        T = T * makeT(frames[k]->xyz, frames[k]->rpy);
        pts[k] = T.block<3,1>(0,3);
        T = T * rz(q(k));
    }
    Eigen::Vector4d footH; footH << footOffset_, 1.0;
    pts[3] = (T * footH).head<3>();
    return pts;
}

bool LegKinematics::inverseKinematics(const Eigen::Vector3d& footTarget, const Eigen::Vector3d& qInit,
                                      Eigen::Vector3d& qOut, double tol, int maxIter) const {
    constexpr double kLambda2 = 1e-4;   // damping^2, bounds steps near singularities
    qOut = qInit;
    for (int it = 0; it < maxIter; ++it) {
        Eigen::Matrix3d J;
        const Eigen::Vector3d e = footTarget - forwardKinematics(qOut, &J);
        if (e.norm() < tol) return true;
        const Eigen::Matrix3d JJt = J * J.transpose() + kLambda2 * Eigen::Matrix3d::Identity();
        qOut += J.transpose() * JJt.ldlt().solve(e);
    }
    return (footTarget - forwardKinematics(qOut)).norm() < tol;
}

std::array<LegKinematics, 4> buildAllLegs() {
    // Exact origins from the URDF (angles in radians).
    JointFrame abd[4] = {
        {{-0.346552, 0.0871865, 0.499727}, {-1.5708, 0, -1.5708}},
        {{-0.346552, -0.0846172, 0.499727}, {1.5708, -0, 1.5708}},
        {{0.237448, 0.0871865, 0.499727}, {1.5708, 0, -1.5708}},
        {{0.237448, -0.0846172, 0.499727}, {-1.5708, -0, 1.5708}},
    };
    JointFrame flex[4] = {
        {{-0.019, -0, -0.0744123}, {1.5708, 1.33288, 1.5708}},
        {{-0.019, 0, -0.0744123}, {-1.5708, 1.31744, -1.5708}},
        {{-0.019, -0, -0.0744123}, {-1.5708, -1.27656, -1.5708}},
        {{-0.019, -0, -0.0744123}, {1.5708, -1.27972, 1.5708}},
    };
    JointFrame knee[4] = {
        {{-0.00169297, 0.26, -0.045}, {-3.14159, 0, -0}},
        {{-0.00169297, -0.26, -0.005}, {-0, 0, 0}},
        {{-0.00169297, 0.26, -0.045}, {3.14159, 0, 0}},
        {{-0.00169297, -0.26, -0.005}, {-0, 0, 0}},
    };
    Eigen::Vector3d footOffset(0.19206, -0.15791, -0.04);
    // <inertial> of leg{n}_upper/_lower/_foot: identical for all 4 legs in the URDF.
    auto sym = [](double ixx, double ixy, double ixz, double iyy, double iyz, double izz) {
        Eigen::Matrix3d I; I << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz; return I;
    };
    std::array<LinkInertial, 3> links = {{
        {0.507359, {0.0186384, 0.0, -0.0635783},
         sym(0.00181356, 0.0, 0.000119632, 0.00127482, 0.0, 0.0011445)},
        {1.13575,  {-0.0136264, 0.0870948, -0.025},
         sym(0.00924521, 0.000782003, 0.0, 0.00202899, 0.0, 0.0105733)},
        {0.752244, {0.0948155, -0.0714168, -0.02},
         sym(0.00215531, 0.00237894, 0.0, 0.00301188, 0.0, 0.00496208)},
    }};
    return { LegKinematics(abd[0], flex[0], knee[0], footOffset, links),
             LegKinematics(abd[1], flex[1], knee[1], footOffset, links),
             LegKinematics(abd[2], flex[2], knee[2], footOffset, links),
             LegKinematics(abd[3], flex[3], knee[3], footOffset, links) };
}
