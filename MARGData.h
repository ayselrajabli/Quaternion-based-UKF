#ifndef AKFSFSIMULATION_MARGDATA_H
#define AKFSFSIMULATION_MARGDATA_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <random>

struct MARGMeasurement{
    double timestamp;
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
    Eigen::Vector3d mag;
};
class MARGData {
public:
    MARGData();
    void reset(); //reset random numbers
    void setNoiseStd(double accel_std,double gyro_std,double mag_std);
    void setBias(const Eigen::Vector3d& gyro_bias);
    MARGMeasurement generateMARGMeasurement(const Eigen::Vector3d& true_gyro,
                                        const Eigen::Vector3d& true_accel,
                                        const Eigen::Vector3d& true_mag,
                                        double timestamp);
private:
    std::mt19937 random_gen;
    double accel_noise_std;
    double gyro_noise_std;
    double mag_noise_std;
    Eigen::Vector3d gyro_bias_;
};


#endif //AKFSFSIMULATION_MARGDATA_H
