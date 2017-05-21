#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
    TODO (Done):
      * Calculate the RMSE here.
    */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    // ... your code here
    if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
        cout << "Input vectors size wrong!!" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for (int i = 0; i < estimations.size(); ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
    /**
    TODO (Done):
      * Calculate a Jacobian here.
    */
    MatrixXd H = MatrixXd(3, 4);

    double px = x_state[0];
    double py = x_state[1];
    double vx = x_state[2];
    double vy = x_state[3];

    double px2 = px * px;
    double py2 = py * py;

    if (fabs(px2 + py2) < 0.0001) {
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return H;
    }

    H(0, 0) = px / sqrt(px2 + py2);
    H(0, 1) = py / sqrt(px2 + py2);
    H(0, 2) = 0;
    H(0, 3) = 0;
    H(1, 0) = -py / (px2 + py2);
    H(1, 1) = px / (px2 + py2);
    H(1, 2) = 0;
    H(1, 3) = 0;
    H(2, 0) = py * (vx * py - vy * px) / sqrt((px2 + py2) * (px2 + py2) * (px2 + py2));
    H(2, 1) = px * (vy * px - vx * py) / sqrt((px2 + py2) * (px2 + py2) * (px2 + py2));
    H(2, 2) = px / sqrt(px2 + py2);
    H(2, 3) = py / sqrt(px2 + py2);

    return H;
}
