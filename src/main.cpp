//
// Created by lihao on 2022/10/3.
//

#include <iostream>
#include <vector>
#include "interface/measurement_package.h"
#include "interface/ground_truth_package.h"
#include "interface/data_process.h"
#include "fusion/sensor_fusion.h"


int main(){
    std::string data_file = "../data/sample-laser-radar-measurement-data-2.txt";
    std::string result_file = "../data/result.txt";

    std::vector<MeasurementPackage> measurements;
    std::vector<GroundTruthPackage> ground_truths;
    if(!load(data_file, measurements, ground_truths, false)) {
        std::cout << "load data failed!" << std::endl;
        return -1;
    }
    std::cout << "measurement length: " << measurements.size() << std::endl;
    std::cout << "ground truth length: " << ground_truths.size() << std::endl;

    // fusion
    SensorFusion fusion(true);
    std::vector<GroundTruthPackage> results;
    for(auto& measurement : measurements) {
        fusion.Process(measurement);
        Eigen::Vector4d X = fusion.kf_.GetX();
        if(!results.empty() && results.back().timestamp_ == measurement.timestamp_) {
            results.back().gt_values_ = X;
        } else {
            GroundTruthPackage result;
            result.timestamp_ = measurement.timestamp_;
            result.gt_values_ = X;
            results.push_back(result);
        }
    }

    // calculate error
    double avg_pos_error = 0;
    double avg_vel_error = 0;
    auto result_iter = results.begin();
    auto ground_truth_iter = ground_truths.begin();
    int count = 0;
    while(result_iter != results.end() && ground_truth_iter != ground_truths.end()) {
        if(result_iter->timestamp_ == ground_truth_iter->timestamp_) {
            Eigen::Vector4d diff = result_iter->gt_values_ - ground_truth_iter->gt_values_;
            double pos_error = diff.segment(0, 2).norm();
            double vel_error = diff.segment(2, 2).norm();
            avg_pos_error += pos_error;
            avg_vel_error += vel_error;
            count++;
            result_iter++;
            ground_truth_iter++;
        } else if(result_iter->timestamp_ < ground_truth_iter->timestamp_) {
            result_iter++;
        } else {
            ground_truth_iter++;
        }
    }
    avg_pos_error /= double(count);
    avg_vel_error /= double(count);
    std::cout << "matched result count " << count << std::endl;
    std::cout << "average position error " << avg_pos_error << std::endl;
    std::cout << "average velocity error " << avg_vel_error << std::endl;

    // save results
    save(result_file, results);

    return 0;
}