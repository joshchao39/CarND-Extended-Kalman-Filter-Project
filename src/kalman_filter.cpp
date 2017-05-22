#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    /**
    TODO (Done):
      * predict the state
    */
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    TODO (Done):
      * update the state by using Kalman Filter equations
    */
    MatrixXd I = MatrixXd::Identity(4, 4);

    VectorXd y = z - H_ * x_;
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + (K * y);
    P_ = (I - K * H_) * P_;
}

double fit_radian(double rad) {
    while (rad < -M_PI) {
        rad += 2 * M_PI;
    }
    while (rad > M_PI) {
        rad -= 2 * M_PI;
    }
    return rad;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
    TODO (Done):
      * update the state by using Extended Kalman Filter equations
    */
    MatrixXd I = MatrixXd::Identity(4, 4);
    double px = x_[0];
    double py = x_[1];
    double vx = x_[2];
    double vy = x_[3];

    MatrixXd H_x = VectorXd(3);
    double px2 = px * px;
    double py2 = py * py;
    H_x << sqrt(px2 + py2),
            atan2(py, px),
            (px * vx + py * vy) / sqrt(px2 + py2);

    VectorXd y = z - H_x;
    y[1] = fit_radian(y[1]);
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + (K * y);
    P_ = (I - K * H_) * P_;
}
