#ifndef AKFSFSIMULATION_KALMANFILTER_H
#define AKFSFSIMULATION_KALMANFILTER_H


#include <Eigen/Dense>
#include <vector>

class KalmanFilter {
public:
    KalmanFilter(double dt);
    void initialize(const Eigen::Vector4d& q0, const Eigen::Vector3d& b0, const Eigen::MatrixXd& P0);
    void predict(const Eigen::Vector3d& gyro);
    void update(const Eigen::Vector3d& accel, const Eigen::Vector3d& mag);

    Eigen::Vector4d getQuaternion() const;

private:
    Eigen::Vector4d q;
    Eigen::Vector3d b;
    Eigen::MatrixXd P;
    double dt_;

    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

    int n;
    double kappa;
    std::vector<Eigen::VectorXd> sigma_points;
    std::vector<double> weights;

    void generateSigmaPoints();
    void propagateSigmaPoints(const Eigen::Vector3d& gyro);
    Eigen::Vector4d computeQuaternionMean(const std::vector<Eigen::Vector4d>& quats, const std::vector<double>& weights);
    Eigen::MatrixXd computeCovariance(const std::vector<Eigen::VectorXd>& X, const Eigen::VectorXd& mean, const Eigen::MatrixXd& Q);
    Eigen::VectorXd measurementModel(const Eigen::Vector4d& q, const Eigen::Vector3d& g, const Eigen::Vector3d& m);
};


#endif //AKFSFSIMULATION_KALMANFILTER_H
