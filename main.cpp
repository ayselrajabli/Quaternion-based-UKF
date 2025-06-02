#include "MARGData.h"
#include "Logger.h"
#include "DataLoader.h"
#include "kalmanfilter.h"
#include "utils.h"
#include <iostream>
#include <cmath>
#include <filesystem>

int main() {
    const std::string csv_path = "marg_log.csv"; //--> change this with your own csv file
    const double dt = 0.01;
    const bool generate_data = false;

    if (generate_data) {
        MARGData marg;
        marg.setNoiseStd(0.02, 0.01, 0.03);
        marg.setBias(Eigen::Vector3d(0.001, 0.002, 0.003));

        Logger::getInstance().init(csv_path);

        Eigen::Vector4d q_true(1, 0, 0, 0);

        for (int i = 0; i < 100; ++i) {
            double t = i * dt;
            Eigen::Vector3d omega_true(0.0, 0.0, 0.05);
            q_true = propagateQuaternionExact(q_true, omega_true, dt);
            q_true.normalize();

            MARGMeasurement m = marg.generateMARGMeasurement(
                    omega_true,
                    Eigen::Vector3d(0.0, 0.0, -9.81),
                    Eigen::Vector3d(0.3, 0.0, 0.5),
                    t
            );

            Logger::getInstance().log(m, q_true, q_true);  // pred == true for now
        }

        Logger::getInstance().close();
        std::cout << "✅ Data generated and written to " << csv_path << std::endl;

    }

    DataLoader loader;
    if (!loader.load(csv_path)) {
        return -1;
    }

    const auto& measurements = loader.getMeasurements();
    KalmanFilter ukf(dt);
    Eigen::Vector4d q0(1, 0, 0, 0);
    Eigen::Vector3d b0 = Eigen::Vector3d::Zero();
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(7, 7) * 0.01;
    ukf.initialize(q0, b0, P0);

    Eigen::Vector4d q_true = q0;
    std::vector<Eigen::Vector4d> preds, truths;

    for (size_t i = 0; i < measurements.size(); ++i) {
        double dt_i = (i == 0) ? 0.0 : measurements[i].timestamp - measurements[i - 1].timestamp;

        ukf.predict(measurements[i].gyro);
        ukf.update(measurements[i].accel, measurements[i].mag);

        Eigen::Vector4d q_pred = ukf.getQuaternion();
        q_true = propagateQuaternionExact(q_true, measurements[i].gyro, dt_i);
        q_true.normalize();

        preds.push_back(q_pred);
        truths.push_back(q_true);
    }

    double rmse = computeRMSE(preds, truths);
    std::cout << "RMSE: " << rmse << std::endl;

    return 0;
}
