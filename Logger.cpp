#include "Logger.h"
#include "../QNUKF/utils.h"

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}

void Logger::init(const std::string& filename) {
    if (!initialized_) {
        file_.open(filename);
        if (file_.is_open()) {
            file_ << "timestamp,"
                  << "accel_x,accel_y,accel_z,"
                  << "gyro_x,gyro_y,gyro_z,"
                  << "mag_x,mag_y,mag_z,"<<"\n";
            initialized_ = true;
        }
    }
}

void Logger::log(const MARGMeasurement& m,
                 const Eigen::Vector4d& q_pred,
                 const Eigen::Vector4d& q_true) {
    if (!initialized_ || !file_.is_open()) return;

    Eigen::Vector3d eul_pred = quaternionToEuler(q_pred);
    Eigen::Vector3d eul_true = quaternionToEuler(q_true);

    file_ << m.timestamp << ","
          << m.accel.x() << "," << m.accel.y() << "," << m.accel.z() << ","
          << m.gyro.x()  << "," << m.gyro.y()  << "," << m.gyro.z()  << ","
          << m.mag.x()   << "," << m.mag.y()   << "," << m.mag.z()<< "\n";

}

void Logger::close() {
    if (file_.is_open()) {
        file_.close();
    }
}

Logger::~Logger() {
    close();
}
