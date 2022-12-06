//
// Created by lihao on 2022/10/3.
//

#ifndef SENSORFUSION_SENSOR_FUSION_H
#define SENSORFUSION_SENSOR_FUSION_H

#include "interface/measurement_package.h"
#include "filter/kalman_filter.h"


class SensorFusion {
public:
    explicit SensorFusion(bool use_extended_kalman_filter=true);

    void Process(MeasurementPackage measurement);
    KalmanFilter kf_;

private:
    bool use_extended_kalman_filter_;
    bool is_initialized_{};
    long long last_timestamp_{};
    Eigen::MatrixXd R_lidar_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd R_fusion_;
    Eigen::MatrixXd H_radar_;
    Eigen::MatrixXd H_lidar_;
};


#endif //SENSORFUSION_SENSOR_FUSION_H
