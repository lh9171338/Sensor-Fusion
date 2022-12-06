//
// Created by lihao on 2022/10/3.
//

#ifndef SENSORFUSION_DATA_PROCESS_H
#define SENSORFUSION_DATA_PROCESS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"
#include "ground_truth_package.h"


bool load(const std::string& filename, std::vector<MeasurementPackage>& measurements,
          std::vector<GroundTruthPackage>& ground_truths, bool use_fusion_data=false);
bool save(const std::string& filename, std::vector<GroundTruthPackage>& results);


#endif //SENSORFUSION_DATA_PROCESS_H
