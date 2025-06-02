#include "MARGData.h"

MARGData::MARGData() :random_gen(std::mt19937()), accel_noise_std(0.0), gyro_noise_std(0.0),
                      mag_noise_std(0.0), gyro_bias_(Eigen::Vector3d::Zero()){}

void MARGData::reset() {
    random_gen = std::mt19937();
}

void MARGData::setNoiseStd(double accel_std, double gyro_std, double mag_std) {
    accel_noise_std = accel_std;
    gyro_noise_std = gyro_std;
    mag_noise_std = mag_std;
}

void MARGData::setBias(const Eigen::Vector3d& gyro_bias) {
    gyro_bias_ = gyro_bias;
}

MARGMeasurement MARGData::generateMARGMeasurement(const Eigen::Vector3d &true_gyro, const Eigen::Vector3d &true_accel,
                                                  const Eigen::Vector3d &true_mag, double timestamp) {
    MARGMeasurement marg_meas;
    std::normal_distribution<double> accel_dist(0.0, accel_noise_std);
    std::normal_distribution<double> gyro_dist(0.0, gyro_noise_std);
    std::normal_distribution<double> mag_dist(0.0, mag_noise_std);


    marg_meas.accel = true_accel + Eigen::Vector3d (accel_dist(random_gen), accel_dist(random_gen), accel_dist(random_gen));
    marg_meas.gyro=true_gyro+gyro_bias_ + Eigen::Vector3d (gyro_dist(random_gen), gyro_dist(random_gen), gyro_dist(random_gen)) ;
    marg_meas.mag = true_mag + Eigen::Vector3d (mag_dist(random_gen), mag_dist(random_gen), mag_dist(random_gen));
    marg_meas.timestamp = timestamp;

    return marg_meas;
}