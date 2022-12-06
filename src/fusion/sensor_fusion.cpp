//
// Created by lihao on 2022/10/3.
//

#include "sensor_fusion.h"


SensorFusion::SensorFusion(bool use_extended_kalman_filter)
{
    use_extended_kalman_filter_ = use_extended_kalman_filter;
    is_initialized_ = false;
    last_timestamp_ = 0.0;

    // Set Lidar's measurement matrix H_lidar_
    H_lidar_ = Eigen::MatrixXd(2, 4);
    H_lidar_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    // Set Radar's measurement matrix H_radar_
    H_radar_ = Eigen::Matrix4d::Identity();

    // Set R. R is provided by Sensor supplier, in sensor datasheet
    // set measurement covariance matrix
    R_lidar_ = Eigen::MatrixXd(2, 2);
    R_lidar_ << 0.0225, 0,
            0, 0.0225;

    // Measurement covariance matrix - radar
    if(use_extended_kalman_filter_) {
        R_radar_ = Eigen::MatrixXd(3, 3);
        R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;
    } else {
        R_radar_ = Eigen::MatrixXd(4, 4);
        R_radar_ << 0.09, 0, 0, 0,
                0, 0.09, 0, 0,
                0, 0, 0.09, 0,
                0, 0, 0, 0.09;
    }

    // Measurement covariance matrix - fusion data
    R_fusion_ = Eigen::MatrixXd(5, 5);
    R_fusion_ << 0.0225, 0, 0, 0, 0,
            0, 0.0225, 0, 0, 0,
            0, 0, 0.09, 0, 0,
            0, 0, 0, 0.0009, 0,
            0, 0, 0, 0, 0.09;
}

void SensorFusion::Process(MeasurementPackage measurement)
{
    // Initialize Kalman filter
    if (!is_initialized_) {
        Eigen::Vector4d X;
        if (measurement.sensor_type_ == MeasurementPackage::LIDAR) {
            double x = measurement.raw_measurements_[0];
            double y = measurement.raw_measurements_[1];
            X << x, y, 0, 0;
        } else if (measurement.sensor_type_ == MeasurementPackage::RADAR) {
            double rho = measurement.raw_measurements_[0];
            double theta = measurement.raw_measurements_[1];
            double v_rho = measurement.raw_measurements_[2];
            double x = rho * cos(theta);
            double y = rho * sin(theta);
            double vx = v_rho * cos(theta);
            double vy = v_rho * sin(theta);
            X << x, y, vx , vy;
        } else if(measurement.sensor_type_ == MeasurementPackage::FUSION) {
            double x = measurement.raw_measurements_[0];
            double y = measurement.raw_measurements_[1];
            double theta = measurement.raw_measurements_[3];
            double v_rho = measurement.raw_measurements_[4];
            double vx = v_rho * cos(theta);
            double vy = v_rho * sin(theta);
            X << x, y, vx, vy;
        }

        // 避免运算时，0作为被除数
        if (fabs(X(0)) < 0.001) {
            X(0) = 0.001;
        }
        if (fabs(X(1)) < 0.001) {
            X(1) = 0.001;
        }
        // 初始化Kalman滤波器
        kf_.Initialization(X);

        // 设置协方差矩阵P
        Eigen::MatrixXd P = Eigen::MatrixXd(4, 4);
        P << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1000.0, 0.0,
                0.0, 0.0, 0.0, 1000.0;
        kf_.SetP(P);

        // 设置过程噪声Q
        Eigen::MatrixXd Q = Eigen::MatrixXd(4, 4);
        Q << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
        kf_.SetQ(Q);

        // 存储第一帧的时间戳，供下一帧数据使用
        last_timestamp_ = measurement.timestamp_;
        is_initialized_ = true;
    } else {
        // 求前后两帧的时间差，数据包中的时间戳单位为微秒，处以1e6，转换为秒
        double delta_t = double(measurement.timestamp_ - last_timestamp_) / 1000000.0; // unit : s
        last_timestamp_ = measurement.timestamp_;

        // 设置状态转移矩阵F
        Eigen::MatrixXd F = Eigen::MatrixXd(4, 4);
        F << 1.0, 0.0, delta_t, 0.0,
                0.0, 1.0, 0.0, delta_t,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
        kf_.SetF(F);

        // 预测
        kf_.Prediction();

        // 更新
        if (measurement.sensor_type_ == MeasurementPackage::LIDAR) {
            kf_.SetH(H_lidar_);
            kf_.SetR(R_lidar_);
            kf_.KFUpdate(measurement.raw_measurements_);
        } else if (measurement.sensor_type_ == MeasurementPackage::RADAR) {
            if(use_extended_kalman_filter_) {
                kf_.SetR(R_radar_);
                kf_.EKFUpdate(measurement.raw_measurements_);
            } else {
                double rho = measurement.raw_measurements_[0];
                double theta = measurement.raw_measurements_[1];
                double v_rho = measurement.raw_measurements_[2];
                double x = rho * cos(theta);
                double y = rho * sin(theta);
                double vx = v_rho * cos(theta);
                double vy = v_rho * sin(theta);
                Eigen::Vector4d raw_measurements(x, y, vx, vy);
                kf_.SetH(H_radar_);
                kf_.SetR(R_lidar_);
                kf_.KFUpdate(raw_measurements);
            }
        } else if (measurement.sensor_type_ == MeasurementPackage::FUSION) {
            kf_.SetR(R_fusion_);
            kf_.EKFUpdate(measurement.raw_measurements_);
        }
    }
}