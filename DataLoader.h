#ifndef AKFSFSIMULATION_DATALOADER_H
#define AKFSFSIMULATION_DATALOADER_H

#include <string>
#include <vector>
#include "MARGData.h"
#include "nlohmann/json.hpp"

class DataLoader {
public:
    bool load(const std::string& filename);
    const std::vector<MARGMeasurement>& getMeasurements() const;

private:
    std::vector<MARGMeasurement> measurements_;
};


#endif //AKFSFSIMULATION_DATALOADER_H
