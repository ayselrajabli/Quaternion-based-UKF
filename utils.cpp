#include "utils.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <vector>

Eigen::Vector4d quaternionProduct(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) {
    double w1 = q1(0), w2 = q2(0);
    Eigen::Vector3d v1 = q1.tail<3>(), v2 = q2.tail<3>();
    Eigen::Vector4d result;
    result(0) = w1 * w2 - v1.dot(v2);
    result.tail<3>() = w1 * v2 + w2 * v1 + v1.cross(v2);
    return result;
}

Eigen::Vector4d quaternionConjugate(const Eigen::Vector4d& q) {
    Eigen::Vector4d qc;
    qc(0) = q(0);
    qc.tail<3>() = -q.tail<3>();
    return qc;
}

Eigen::Matrix4d omegaMatrix(const Eigen::Vector3d& omega) {
    Eigen::Matrix3d skew;
    skew <<    0, -omega(2),  omega(1),
            omega(2),     0, -omega(0),
            -omega(1), omega(0),     0;
    Eigen::Matrix4d Omega = Eigen::Matrix4d::Zero();
    Omega.block<1, 3>(0, 1) = -omega.transpose();
    Omega.block<3, 1>(1, 0) =  omega;
    Omega.block<3, 3>(1, 1) = -skew;
    return Omega;
}

Eigen::Vector4d normQuaternion(const Eigen::Vector4d& q) {
    return q / q.norm();
}

Eigen::Vector4d propagateQuaternionExact(const Eigen::Vector4d& q, const Eigen::Vector3d& omega, double dt) {
    Eigen::Matrix4d Omega = omegaMatrix(omega);
    Eigen::Matrix4d expOmega = (0.5 * dt * Omega).exp();
    return normQuaternion(expOmega * q);
}

Eigen::Vector4d smallAngleQuaternion(const Eigen::Vector3d& dphi) {
    double angle = dphi.norm();
    Eigen::Vector4d dq;
    if (angle < 1e-6) {
        dq << 1.0, 0.5 * dphi;
    } else {
        Eigen::Vector3d axis = dphi / angle;
        dq(0) = std::cos(angle / 2);
        dq.tail<3>() = axis * std::sin(angle / 2);
    }
    return dq;
}

Eigen::Vector3d quaternionLog(const Eigen::Vector4d& q) {
    Eigen::Vector3d v = q.tail<3>();
    double w = q(0);
    double norm_v = v.norm();
    if (norm_v < 1e-10) {
        return Eigen::Vector3d::Zero();
    } else {
        double angle = 2.0 * std::atan2(norm_v, w);
        return angle * v.normalized();
    }
}

Eigen::Vector4d quaternionExp(const Eigen::Vector3d& v) {
    double theta = v.norm();
    Eigen::Vector4d q;
    if (theta < 1e-10) {
        q << 1.0, 0.5 * v;
    } else {
        Eigen::Vector3d axis = v / theta;
        q(0) = std::cos(theta / 2.0);
        q.tail<3>() = axis * std::sin(theta / 2.0);
    }
    return q;
}

Eigen::Vector3d quaternionToEuler(const Eigen::Vector4d& q) {
    Eigen::Quaterniond quat(q(0), q(1), q(2), q(3));
    return quat.toRotationMatrix().eulerAngles(2, 1, 0).reverse(); // Roll-Pitch-Yaw
}

Eigen::Vector3d quatToAxisAngle(const Eigen::Vector4d& q) {
    Eigen::Vector4d qn = q.normalized();
    double angle = 2 * std::acos(qn(0));
    double s = std::sqrt(1 - qn(0) * qn(0));
    if (s < 1e-8) return Eigen::Vector3d::Zero();  // near zero rotation
    return angle * qn.tail(3) / s;
}

double computeRMSE(const std::vector<Eigen::Vector4d>& est, const std::vector<Eigen::Vector4d>& gt) {
    if (est.size() != gt.size() || est.empty()) {
        return -1.0;
    }

    double total = 0.0;
    for (size_t i = 0; i < est.size(); ++i) {
        Eigen::Vector4d dq = quaternionProduct(quaternionConjugate(gt[i]), est[i]);
        Eigen::Vector3d dphi = quaternionLog(dq);
        total += dphi.squaredNorm();
    }
    return std::sqrt(total / est.size());
}