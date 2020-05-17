#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * Calculate the RMSE here.
   */
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
        std::cout << "Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }

    // accumulate squared residuals
    for (unsigned int i=0; i < estimations.size(); ++i) {

        VectorXd residual = estimations[i] - ground_truth[i];

        // coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    // calculate the mean
    rmse = rmse/estimations.size();

    // calculate the squared root
    rmse = rmse.array().sqrt();

    // return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate a Jacobian here.
   */

    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    if (fabs(px*px + py*py) < 0.0001) {
        throw std::runtime_error("CalculateJacobian () - Error - Division by Zero");
    }


    MatrixXd Hj = MatrixXd(3,4);
    Hj << px/pow((px*px + py*py),0.5),               py/pow((px*px + py*py),0.5),               0,                           0,
            -py/(px*px + py*py),                       px/(px*px + py*py),                        0,                           0,
            py*(vx*py-vy*px)/pow((px*px + py*py),1.5), px*(vy*px-vx*py)/pow((px*px + py*py),1.5), px/pow((px*px + py*py),0.5), py/pow((px*px + py*py),0.5);
    return Hj;
}
