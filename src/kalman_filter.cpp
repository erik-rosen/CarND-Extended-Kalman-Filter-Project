#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {
    x_ = VectorXd(4);
    P_ = MatrixXd(4, 4);
    F_ = MatrixXd(4, 4);
    H_ = MatrixXd();
    R_ = MatrixXd();
    Q_ = MatrixXd(4, 4);
    F_ = MatrixXd(4, 4);
}

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
   * predict the state
   */
   x_ = F_ * x_;
   P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
   VectorXd y = z - H_ * x_;
   MatrixXd S = H_ * P_ * H_.transpose() + R_;
   MatrixXd K = P_ * H_.transpose() * S.inverse();
   x_ = x_ + K * y;
   P_ = (MatrixXd::Identity(4, 4) - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   *update the state by using Extended Kalman Filter equations
   */
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    float atanxy;
    VectorXd h_x = VectorXd(3);

    h_x << sqrt(px*px + py*py), atan2(py, px), (px*vx + py*vy)/sqrt(px*px + py*py);
    VectorXd y = z - h_x;
    //std::cout << "h(x): " << h_x << std::endl;
    //std::cout << "y: " << y << std::endl;

    //Normalize phi to be between pi and -pi
    while (y(1) > M_PI || y(1) < - M_PI ){
        if (y(1) > M_PI){
            y(1) = y(1) - M_PI*2.0;
        }
        if (y(1) < -M_PI){
            y(1) = y(1) + M_PI*2.0;
        }
    }

    //std::cout << "Phi: " << y(1) << std::endl;

    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (MatrixXd::Identity(4, 4) - K * H_) * P_;
}
