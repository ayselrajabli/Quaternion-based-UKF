#include "../QNUKF/kalmanfilter.h"
#include "../QNUKF/utils.h"

KalmanFilter::KalmanFilter(double dt)
        : dt_(dt), n(6), kappa(0.0)
{
    Q = Eigen::MatrixXd::Identity(7, 7) * 1e-5;
    R = Eigen::MatrixXd::Identity(6, 6) * 1e-2;
    weights.resize(2 * n + 1);
    weights[0] = kappa / (n + kappa);
    for (int i = 1; i < 2 * n + 1; ++i)
        weights[i] = 1.0 / (2 * (n + kappa));
}

void KalmanFilter::initialize(const Eigen::Vector4d& q0, const Eigen::Vector3d& b0, const Eigen::MatrixXd& P0) {
    q = q0;
    b = b0;
    P = P0;
}

Eigen::Vector4d KalmanFilter::getQuaternion() const {
    return q;
}


void KalmanFilter::generateSigmaPoints() {
    Eigen::MatrixXd S = P.llt().matrixL();
    sigma_points.clear();

    // First sigma point is the nominal state
    Eigen::VectorXd x0(7);
    x0.head<4>() = q;
    x0.tail<3>() = b;
    sigma_points.push_back(x0);

    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd delta = std::sqrt(n + kappa) * S.col(i);


        Eigen::Vector3d dphi = delta.head<3>();
        Eigen::Vector4d dq = quaternionExp(0.5 * dphi);
        Eigen::Vector4d q_sigma = quaternionProduct(dq, q);
        Eigen::VectorXd x_plus(7);
        x_plus.head<4>() = q_sigma;
        x_plus.tail<3>() = b + delta.tail<3>();
        sigma_points.push_back(x_plus);


        delta = -delta;
        dphi = delta.head<3>();
        dq = quaternionExp(0.5 * dphi);
        q_sigma = quaternionProduct(dq, q);
        Eigen::VectorXd x_minus(7);
        x_minus.head<4>() = q_sigma;
        x_minus.tail<3>() = b + delta.tail<3>();
        sigma_points.push_back(x_minus);
    }
}

void KalmanFilter::propagateSigmaPoints(const Eigen::Vector3d& gyro) {
    for (auto& x : sigma_points) {
        Eigen::Vector4d q = x.head<4>();
        Eigen::Vector3d b = x.tail<3>();
        Eigen::Vector3d omega_corr = gyro - b;
        Eigen::Vector4d q_next = propagateQuaternionExact(q, omega_corr, dt_);
        x.head<4>() = q_next;
    }
}

Eigen::Vector4d KalmanFilter::computeQuaternionMean(const std::vector<Eigen::Vector4d>& quats, const std::vector<double>& weights) {
    Eigen::Vector4d q_mean = quats[0];
    for (int iter = 0; iter < 10; ++iter) {
        Eigen::Vector3d delta = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < quats.size(); ++i) {
            Eigen::Vector4d q_err = quaternionProduct(quats[i], quaternionConjugate(q_mean));
            Eigen::Vector3d log_q = quaternionLog(q_err);
            delta += weights[i] * log_q;
        }
        if (delta.norm() < 1e-6)
            break;
        q_mean = quaternionProduct(quaternionExp(delta), q_mean);
        q_mean.normalize();
    }
    return q_mean;
}
void KalmanFilter::predict(const Eigen::Vector3d& gyro) {
    generateSigmaPoints();
    propagateSigmaPoints(gyro);

    std::vector<Eigen::Vector4d> quats;
    std::vector<Eigen::Vector3d> biases;
    for (const auto& x : sigma_points) {
        quats.push_back(x.head<4>());
        biases.push_back(x.tail<3>());
    }

    q = computeQuaternionMean(quats, weights);

    b = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < biases.size(); ++i)
        b += weights[i] * biases[i];

    Eigen::VectorXd x_mean(7);
    x_mean.head<4>() = q;
    x_mean.tail<3>() = b;

    P = Eigen::MatrixXd::Zero(7, 7);
    for (size_t i = 0; i < sigma_points.size(); ++i) {
        Eigen::VectorXd dx(7);
        Eigen::Vector4d dq = quaternionProduct(quaternionConjugate(q), sigma_points[i].head<4>());
        dx.head<3>() = quaternionLog(dq);
        dx.tail<3>() = sigma_points[i].tail<3>() - b;
        P += weights[i] * dx * dx.transpose();
    }
    P += Q;
}

Eigen::VectorXd KalmanFilter::measurementModel(const Eigen::Vector4d& q, const Eigen::Vector3d& g, const Eigen::Vector3d& m) {
    Eigen::Vector4d g_quat, m_quat;
    g_quat << 0, g;
    m_quat << 0, m;

    Eigen::Vector4d q_conj = quaternionConjugate(q);
    Eigen::Vector4d g_rot = quaternionProduct(quaternionProduct(q, g_quat), q_conj);
    Eigen::Vector4d m_rot = quaternionProduct(quaternionProduct(q, m_quat), q_conj);

    Eigen::VectorXd y(6);
    y.head<3>() = g_rot.tail<3>();
    y.tail<3>() = m_rot.tail<3>();
    return y;
}

void KalmanFilter::update(const Eigen::Vector3d& accel, const Eigen::Vector3d& mag) {
    const Eigen::Vector3d g(0, 0, -9.81);
    const Eigen::Vector3d m(1, 0, 0);

    std::vector<Eigen::VectorXd> Y_sigma;
    for (const auto& x : sigma_points) {
        Y_sigma.push_back(measurementModel(x.head<4>(), g, m));
    }

    Eigen::VectorXd y_mean = Eigen::VectorXd::Zero(6);
    for (size_t i = 0; i < Y_sigma.size(); ++i)
        y_mean += weights[i] * Y_sigma[i];

    Eigen::MatrixXd P_yy = Eigen::MatrixXd::Zero(6, 6);
    for (size_t i = 0; i < Y_sigma.size(); ++i) {
        Eigen::VectorXd dy = Y_sigma[i] - y_mean;
        P_yy += weights[i] * dy * dy.transpose();
    }
    P_yy += R;

    Eigen::MatrixXd P_xy = Eigen::MatrixXd::Zero(7, 6);
    for (size_t i = 0; i < sigma_points.size(); ++i) {
        Eigen::VectorXd dx(7);
        dx.head<3>() = quaternionLog(quaternionProduct(quaternionConjugate(q), sigma_points[i].head<4>()));
        dx.tail<3>() = sigma_points[i].tail<3>() - b;
        Eigen::VectorXd dy = Y_sigma[i] - y_mean;
        P_xy += weights[i] * dx * dy.transpose();
    }

    Eigen::MatrixXd K = P_xy * P_yy.inverse();

    Eigen::VectorXd y_meas(6);
    y_meas.head<3>() = accel;
    y_meas.tail<3>() = mag;
    Eigen::VectorXd innovation = y_meas - y_mean;

    Eigen::Vector3d dq_vec = K.topRows<3>() * innovation;
    Eigen::Vector4d dq = quaternionExp(dq_vec);
    q = quaternionProduct(dq, q);
    q.normalize();

    b += K.bottomRows<3>() * innovation;

    P -= K * P_yy * K.transpose();
}



