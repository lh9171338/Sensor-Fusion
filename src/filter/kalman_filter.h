//
// Created by lihao on 2022/10/3.
//

#ifndef SENSORFUSION_KALMAN_FILTER_H
#define SENSORFUSION_KALMAN_FILTER_H


#include <iostream>
#include "Eigen/Dense"


class KalmanFilter {
public:
    KalmanFilter();

    void Initialization(Eigen::VectorXd X);
    bool IsInitialized();

    void SetF(Eigen::MatrixXd F);
    void SetP(Eigen::MatrixXd P);
    void SetQ(Eigen::MatrixXd Q);
    void SetH(Eigen::MatrixXd H);
    void SetR(Eigen::MatrixXd R);

    void Prediction();
    void KFUpdate(Eigen::VectorXd Z);
    void EKFUpdate(Eigen::VectorXd Z);
    Eigen::VectorXd GetX();

private:

    Eigen::MatrixXd CalculateJacobianMatrix();

    // flag of initialization
    bool is_initialized_;
    // state vector
    Eigen::VectorXd X_;
    // state covariance matrix
    Eigen::MatrixXd P_;
    // state transition matrix
    Eigen::MatrixXd F_;
    // process covariance matrix
    Eigen::MatrixXd Q_;
    // measurement matrix
    Eigen::MatrixXd H_;
    // measurement covariance matrix
    Eigen::MatrixXd R_;
};


#endif //SENSORFUSION_KALMAN_FILTER_H
