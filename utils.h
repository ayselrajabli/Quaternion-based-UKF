#ifndef AKFSFSIMULATION_UTILS_H
#define AKFSFSIMULATION_UTILS_H

#include <Eigen/Dense>
#include <vector>

Eigen::Vector4d quaternionProduct(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2);
Eigen::Vector4d quaternionConjugate(const Eigen::Vector4d& q);
Eigen::Matrix4d omegaMatrix(const Eigen::Vector3d& omega);
Eigen::Vector4d normQuaternion(const Eigen::Vector4d& q);
Eigen::Vector4d propagateQuaternionExact(const Eigen::Vector4d& q, const Eigen::Vector3d& omega, double dt);
Eigen::Vector4d smallAngleQuaternion(const Eigen::Vector3d& dphi);
Eigen::Vector3d quaternionLog(const Eigen::Vector4d& q);
Eigen::Vector4d quaternionExp(const Eigen::Vector3d& v);
Eigen::Vector3d quaternionToEuler(const Eigen::Vector4d& q);
double computeRMSE(const std::vector<Eigen::Vector4d>& preds, const std::vector<Eigen::Vector4d>& gts);

#endif //AKFSFSIMULATION_UTILS_H
