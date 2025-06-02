#include "DataLoader.h"
#include <fstream>
#include <sstream>
#include <iostream>


bool DataLoader::load(const std::string& filename) {
    measurements_.clear();

    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> values;

        while (std::getline(ss, cell, ',')) {
            if (!cell.empty()) {
                try {
                    values.push_back(std::stod(cell));
                }
                catch (...) {
                    return false;
                }
            }
        }

        if (values.size() < 10) continue;  // timestamp + 3 accel + 3 gyro + 3 mag

        MARGMeasurement m;
        m.timestamp = values[0];
        m.accel = Eigen::Vector3d(values[1], values[2], values[3]);
        m.gyro  = Eigen::Vector3d(values[4], values[5], values[6]);
        m.mag   = Eigen::Vector3d(values[7], values[8], values[9]);

        measurements_.push_back(m);
    }
    return true;
}

const std::vector<MARGMeasurement>& DataLoader::getMeasurements() const {
    return measurements_;
}