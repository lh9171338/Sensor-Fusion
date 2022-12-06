//
// Created by lihao on 2022/10/3.
//

#include "kalman_filter.h"


KalmanFilter::KalmanFilter()
{
    is_initialized_ = false;
}

void KalmanFilter::Initialization(Eigen::VectorXd X)
{
    X_ = X;
}

bool KalmanFilter::IsInitialized()
{
    return is_initialized_;
}

void KalmanFilter::SetF(Eigen::MatrixXd F)
{
    F_ = F;
}

void KalmanFilter::SetP(Eigen::MatrixXd P)
{
    P_ = P;
}

void KalmanFilter::SetQ(Eigen::MatrixXd Q)
{
    Q_ = Q;
}

void KalmanFilter::SetH(Eigen::MatrixXd H)
{
    H_ = H;
}

void KalmanFilter::SetR(Eigen::MatrixXd R)
{
    R_ = R;
}

void KalmanFilter::Prediction()
{
    X_ = F_ * X_;
    Eigen::MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::KFUpdate(Eigen::VectorXd Z)
{
    Eigen::VectorXd Y = Z - H_ * X_;
    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd S = H_ * P_ * Ht + R_;
    Eigen::MatrixXd K =  P_ * Ht * S.inverse();
    X_ = X_ + K * Y;
    int x_size = X_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::EKFUpdate(Eigen::VectorXd Z)
{
    int z_size = Z.size();
    // get state parameters
    double x = X_(0);
    double y = X_(1);
    double vx = X_(2);
    double vy = X_(3);

    double rho = sqrt(x * x + y * y);
    double theta = atan2(y, x);
    double v_rho = (x * vx + y * vy) / rho;
    Eigen::VectorXd h = Eigen::VectorXd::Zero(5);
    h << x, y, rho, theta, v_rho;
    Eigen::VectorXd Y = Z - h.bottomRows(z_size);

    Eigen::MatrixXd H = CalculateJacobianMatrix();
    H_ = H.bottomRows(z_size);
    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd S = H_ * P_ * Ht + R_;
    Eigen::MatrixXd K =  P_ * Ht * S.inverse();
    X_ = X_ + K * Y;
    int x_size = X_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

Eigen::VectorXd KalmanFilter::GetX()
{
    return X_;
}

Eigen::MatrixXd KalmanFilter::CalculateJacobianMatrix()
{
    // get state parameters
    double x = X_(0);
    double y = X_(1);
    double vx = X_(2);
    double vy = X_(3);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(5, 4);

    // pre-compute a set of terms to avoid repeated calculation
    double c1 = x * x + y * y;
    double c2 = sqrt(c1);
    double c3 = (c1 * c2);
    double c4 = x * vy - y * vx;

    // Check division by zero
    if(fabs(c1) < 0.0001){
        return H;
    }

    H << 1, 0, 0, 0,
            0, 1, 0, 0,
            x / c2, y / c2, 0, 0,
            -y / c1, x / c1, 0, 0,
            -y * c4 / c3, x * c4 / c3, x / c2, y / c2;
    return H;
}
