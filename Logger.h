#ifndef AKFSFSIMULATION_LOGGER_H
#define AKFSFSIMULATION_LOGGER_H



#include "MARGData.h"
#include "nlohmann/json.hpp"
#include <fstream>
#include <string>

class Logger {
public:
    static Logger& getInstance();

    void init(const std::string& filename);
    void log(const MARGMeasurement& m,
             const Eigen::Vector4d& q_pred,
             const Eigen::Vector4d& q_true);
    void close();

private:
    Logger() = default;
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    std::ofstream file_;
    nlohmann::json json_log_;
    bool initialized_ = false;
};



#endif //AKFSFSIMULATION_LOGGER_H
