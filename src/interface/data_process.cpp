//
// Created by lihao on 2022/10/3.
//

#include "data_process.h"


bool load(const std::string& filename, std::vector<MeasurementPackage>& measurements,
          std::vector<GroundTruthPackage>& ground_truths, bool use_fusion_data) {
    measurements.clear();
    ground_truths.clear();
    std::ifstream fs(filename, std::ios::in);
    if (!fs.is_open()) {
        std::cout << "can't open file " << filename << std::endl;
        return false;
    }
    std::string line;
    while(std::getline(fs, line)) {
        std::istringstream iss(line);
        std::string sensor_type;
        iss >> sensor_type;
        if(sensor_type == "L") {
            long long ts;
            double x, y, gt_x, gt_y, gt_vx, gt_vy;
            iss >> x >> y >> ts >> gt_x >> gt_y >> gt_vx >> gt_vy;
            if(use_fusion_data && !measurements.empty() && measurements.back().timestamp_ == ts) {
                MeasurementPackage& measurement = measurements.back();
                Eigen::VectorXd raw_measurements = Eigen::VectorXd::Zero(5);
                raw_measurements << Eigen::Vector2d(x, y), measurement.raw_measurements_;
                measurement.sensor_type_ = MeasurementPackage::FUSION;
                measurement.raw_measurements_ = raw_measurements;
            } else {
                MeasurementPackage measurement;
                measurement.sensor_type_ = MeasurementPackage::LIDAR;
                measurement.timestamp_ = ts;
                measurement.raw_measurements_ = Eigen::Vector2d(x, y);
                measurements.push_back(measurement);
            }

            GroundTruthPackage ground_truth;
            ground_truth.timestamp_ = ts;
            ground_truth.gt_values_ = Eigen::Vector4d(gt_x, gt_y, gt_vx, gt_vy);
            ground_truths.push_back(ground_truth);
        } else if(sensor_type == "R") {
            long long ts;
            double rho, theta, v, gt_x, gt_y, gt_vx, gt_vy;
            iss >> rho >> theta >> v >> ts >> gt_x >> gt_y >> gt_vx >> gt_vy;
            if(use_fusion_data && !measurements.empty() && measurements.back().timestamp_ == ts) {
                MeasurementPackage& measurement = measurements.back();
                Eigen::VectorXd raw_measurements = Eigen::VectorXd::Zero(5);
                raw_measurements << measurement.raw_measurements_, Eigen::Vector3d(rho, theta, v);
                measurement.sensor_type_ = MeasurementPackage::FUSION;
                measurement.raw_measurements_ = raw_measurements;
            } else {
                MeasurementPackage measurement;
                measurement.sensor_type_ = MeasurementPackage::RADAR;
                measurement.timestamp_ = ts;
                measurement.raw_measurements_ = Eigen::Vector3d(rho, theta, v);
                measurements.push_back(measurement);
            }

            GroundTruthPackage ground_truth;
            ground_truth.timestamp_ = ts;
            ground_truth.gt_values_ = Eigen::Vector4d(gt_x, gt_y, gt_vx, gt_vy);
            ground_truths.push_back(ground_truth);
        } else {
            std::cout << "not support type: " << sensor_type << std::endl;
        }
    }
    fs.close();

    return true;
}

bool save(const std::string& filename, std::vector<GroundTruthPackage>& results) {
    std::ofstream fs(filename, std::ios::out);
    if (!fs.is_open()) {
        std::cout << "can't open file " << filename << std::endl;
        return false;
    }
    for(auto& result : results) {
        fs << result << std::endl;
    }
    fs.close();

    return true;
}