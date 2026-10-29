#include "quadru_control/floating_base_model.hpp"
#include "quadru_control/spatial.hpp"
#include <cmath>

namespace {

// Same as rpy2R/rz in leg_kinematics.cpp: Xup must reproduce the FK chain.
Eigen::Matrix3d rpy2R(const Eigen::Vector3d& r) {
    double cr = std::cos(r(0)), sr = std::sin(r(0));
    double cp = std::cos(r(1)), sp = std::sin(r(1));
    double cy = std::cos(r(2)), sy = std::sin(r(2));
    Eigen::Matrix3d Rz_, Ry_, Rx_;
    Rz_ << cy, -sy, 0,  sy, cy, 0,  0, 0, 1;
    Ry_ << cp, 0, sp,   0, 1, 0,   -sp, 0, cp;
    Rx_ << 1, 0, 0,   0, cr, -sr,   0, sr, cr;
    return Rz_ * Ry_ * Rx_;
}

Eigen::Matrix3d rz(double q) {
    double c = std::cos(q), s = std::sin(q);
    Eigen::Matrix3d R;
    R << c, -s, 0,  s, c, 0,  0, 0, 1;
    return R;
}

}  // namespace

FloatingBaseModel::FloatingBaseModel(double trunkMass, const Eigen::Vector3d& trunkCom,
                                      const Eigen::Matrix3d& trunkInertia,
                                      const std::array<LegKinematics, 4>& legs) {
    bodies_[0].parent = -1;
    bodies_[0].I = spatial::spatialInertia(trunkMass, trunkCom, trunkInertia);

    for (int leg = 0; leg < 4; ++leg) {
        auto frames = legs[leg].jointFrames();
        const auto& links = legs[leg].linkInertials();
        for (int k = 0; k < 3; ++k) {
            int i = 1 + 3 * leg + k;              // this body's index
            jointFrame_[i - 1] = frames[k];
            bodies_[i].parent = (k == 0) ? 0 : i - 1;
            bodies_[i].I = spatial::spatialInertia(links[k].mass, links[k].com, links[k].inertia);
        }
    }
}

void FloatingBaseModel::computeXup(const Eigen::Matrix<double, 12, 1>& qJoints,
                                    std::array<Eigen::Matrix<double, 6, 6>, kNb>& Xup) const {
    for (int i = 1; i <= 12; ++i) {
        const JointFrame& f = jointFrame_[i - 1];
        Eigen::Matrix3d Efixed = rpy2R(f.rpy);
        Eigen::Matrix3d Etotal = Efixed * rz(qJoints(i - 1));   // child -> parent rotation
        // Xup maps motion vectors parent -> child, hence Etotal^T.
        Xup[i] = spatial::xlateRotate(Etotal.transpose(), f.xyz);
    }
}

Eigen::Matrix<double, FloatingBaseModel::kNv, FloatingBaseModel::kNv>
FloatingBaseModel::massMatrix(const Eigen::Matrix<double, 12, 1>& qJoints) const {
    std::array<Eigen::Matrix<double, 6, 6>, kNb> Xup;
    computeXup(qJoints, Xup);

    std::array<Eigen::Matrix<double, 6, 6>, kNb> IC;
    for (int i = 0; i < kNb; ++i) IC[i] = bodies_[i].I;
    for (int i = kNb - 1; i >= 1; --i) IC[bodies_[i].parent] += Xup[i].transpose() * IC[i] * Xup[i];

    Eigen::Matrix<double, kNv, kNv> M = Eigen::Matrix<double, kNv, kNv>::Zero();
    M.block<6, 6>(0, 0) = IC[0];

    for (int i = 1; i <= 12; ++i) {
        Eigen::Matrix<double, 6, 1> Fi = IC[i].col(2);   // IC[i] * S_i, S_i = local z rotation
        M(6 + (i - 1), 6 + (i - 1)) = Fi(2);
        int j = i;
        while (bodies_[j].parent != -1) {
            Fi = Xup[j].transpose() * Fi;
            int p = bodies_[j].parent;
            if (p == 0) {
                M.block<6, 1>(0, 6 + (i - 1)) = Fi;
                M.block<1, 6>(6 + (i - 1), 0) = Fi.transpose();
            } else {
                M(6 + (p - 1), 6 + (i - 1)) = Fi(2);
                M(6 + (i - 1), 6 + (p - 1)) = Fi(2);
            }
            j = p;
        }
    }
    return M;
}

Eigen::Matrix<double, FloatingBaseModel::kNv, 1>
FloatingBaseModel::biasForces(const Eigen::Matrix3d& R0, const Eigen::Matrix<double, 12, 1>& qJoints,
                               const Eigen::Matrix<double, 6, 1>& vBase,
                               const Eigen::Matrix<double, 12, 1>& vJoints) const {
    return inverseDynamics(R0, qJoints, vBase, vJoints,
                            Eigen::Matrix<double, 6, 1>::Zero(), Eigen::Matrix<double, 12, 1>::Zero());
}

void FloatingBaseModel::propagateVA(const std::array<Eigen::Matrix<double, 6, 6>, kNb>& Xup,
                                     const Eigen::Matrix<double, 6, 1>& vBase, const Eigen::Matrix<double, 12, 1>& vJoints,
                                     const Eigen::Matrix<double, 6, 1>& aBase, const Eigen::Matrix<double, 12, 1>& aJoints,
                                     std::array<Eigen::Matrix<double, 6, 1>, kNb>& v,
                                     std::array<Eigen::Matrix<double, 6, 1>, kNb>& a) const {
    v[0] = vBase;
    a[0] = aBase;
    for (int i = 1; i <= 12; ++i) {
        int p = bodies_[i].parent;
        Eigen::Matrix<double, 6, 1> Sqd = Eigen::Matrix<double, 6, 1>::Zero();
        Sqd(2) = vJoints(i - 1);
        Eigen::Matrix<double, 6, 1> Sqdd = Eigen::Matrix<double, 6, 1>::Zero();
        Sqdd(2) = aJoints(i - 1);
        v[i] = Xup[i] * v[p] + Sqd;
        a[i] = Xup[i] * a[p] + Sqdd + spatial::crossMotion(v[i]) * Sqd;
    }
}

Eigen::Matrix<double, FloatingBaseModel::kNv, 1>
FloatingBaseModel::inverseDynamics(const Eigen::Matrix3d& R0, const Eigen::Matrix<double, 12, 1>& qJoints,
                                    const Eigen::Matrix<double, 6, 1>& vBase,
                                    const Eigen::Matrix<double, 12, 1>& vJoints,
                                    const Eigen::Matrix<double, 6, 1>& aBase,
                                    const Eigen::Matrix<double, 12, 1>& aJoints) const {
    std::array<Eigen::Matrix<double, 6, 6>, kNb> Xup;
    computeXup(qJoints, Xup);

    const Eigen::Vector3d gWorld(0, 0, -9.81);
    Eigen::Matrix<double, 6, 1> a0 = aBase;
    a0.tail<3>() -= R0.transpose() * gWorld;   // gravity as a base acceleration

    std::array<Eigen::Matrix<double, 6, 1>, kNb> v, a, f;
    propagateVA(Xup, vBase, vJoints, a0, aJoints, v, a);

    for (int i = 0; i < kNb; ++i)
        f[i] = bodies_[i].I * a[i] + spatial::crossForce(v[i]) * (bodies_[i].I * v[i]);
    for (int i = 12; i >= 1; --i) f[bodies_[i].parent] += Xup[i].transpose() * f[i];

    Eigen::Matrix<double, kNv, 1> tau;
    tau.segment<6>(0) = f[0];
    for (int i = 1; i <= 12; ++i) tau(6 + (i - 1)) = f[i](2);
    return tau;
}

std::array<Eigen::Matrix3d, FloatingBaseModel::kNb>
FloatingBaseModel::computeRToTrunk(const Eigen::Matrix<double, 12, 1>& qJoints) const {
    std::array<Eigen::Matrix3d, kNb> R;
    R[0] = Eigen::Matrix3d::Identity();
    for (int i = 1; i <= 12; ++i) {
        const JointFrame& f = jointFrame_[i - 1];
        Eigen::Matrix3d childToParent = rpy2R(f.rpy) * rz(qJoints(i - 1));   // as Etotal in computeXup
        R[i] = R[bodies_[i].parent] * childToParent;
    }
    return R;
}

Eigen::Matrix<double, 3, FloatingBaseModel::kNv>
FloatingBaseModel::contactJacobian(const Eigen::Matrix<double, 12, 1>& qJoints,
                                    const std::array<LegKinematics, 4>& legs, int leg) const {
    Eigen::Matrix3d Jleg;
    Eigen::Vector3d rFoot = legs[leg].forwardKinematics(qJoints.segment<3>(3 * leg), &Jleg);
    Eigen::Matrix<double, 3, kNv> Jc = Eigen::Matrix<double, 3, kNv>::Zero();
    Jc.block<3, 3>(0, 0) = -spatial::skew(rFoot);
    Jc.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    Jc.block<3, 3>(0, 6 + 3 * leg) = Jleg;
    return Jc;
}

std::array<Eigen::Vector3d, 4>
FloatingBaseModel::footBiasAcceleration(const Eigen::Matrix<double, 12, 1>& qJoints,
                                         const std::array<LegKinematics, 4>& legs,
                                         const Eigen::Matrix<double, 6, 1>& vBase,
                                         const Eigen::Matrix<double, 12, 1>& vJoints) const {
    std::array<Eigen::Matrix<double, 6, 6>, kNb> Xup;
    computeXup(qJoints, Xup);
    std::array<Eigen::Matrix<double, 6, 1>, kNb> v, a;
    propagateVA(Xup, vBase, vJoints, Eigen::Matrix<double, 6, 1>::Zero(),
                Eigen::Matrix<double, 12, 1>::Zero(), v, a);
    std::array<Eigen::Matrix3d, kNb> Rt = computeRToTrunk(qJoints);

    std::array<Eigen::Vector3d, 4> out;
    for (int leg = 0; leg < 4; ++leg) {
        int footBody = 3 + 3 * leg;   // foot link of this leg
        // v/a are at the foot link origin (knee), in its own frame; transport them to the contact
        // point footOffset_, as contactJacobian() does.
        Eigen::Vector3d omegaT = Rt[footBody] * v[footBody].head<3>();
        Eigen::Vector3d vlinT = Rt[footBody] * v[footBody].tail<3>();
        Eigen::Vector3d alinT = Rt[footBody] * a[footBody].tail<3>();
        Eigen::Vector3d alphaT = Rt[footBody] * a[footBody].head<3>();
        Eigen::Vector3d originBias = alinT + omegaT.cross(vlinT);

        Eigen::Vector3d d = Rt[footBody] * legs[leg].footOffset();
        out[leg] = originBias + alphaT.cross(d) + omegaT.cross(omegaT.cross(d));
    }
    return out;
}
