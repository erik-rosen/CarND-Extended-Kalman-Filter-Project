#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1;


  noise_ax_ = 9;
  noise_ay_ = 9;


}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates
      // and initialize state.
      // Order is: ro,theta, ro_dot
      float ro = measurement_pack.raw_measurements_(0);
      float theta = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);

      float px = ro * cos(theta);
      float py = ro * sin(theta);
      float vx = ro_dot * cos(theta);
      float vy = ro_dot * sin(theta);
      ekf_.x_ << px, py, vx, vy;
      ekf_.P_ <<
              1,    0,    0,    0,
              0,    1,    0,    0,
              0,    0,    1,    0,
              0,    0,    0,    1;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;
      ekf_.P_ <<
              1,    0,    0,    0,
              0,    1,    0,    0,
              0,    0,    1000, 0,
              0,    0,    0,    1000;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction:
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Using noise_ax_ = 9 and noise_ay_ = 9 for your Q matrix.
   */

  double dt_s = double((measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0);
  previous_timestamp_ = measurement_pack.timestamp_;

  //We reset if time goes backward (usually if the simulation is reset) or more than 10s has elapsed
  if (dt_s<0||dt_s>10){
      is_initialized_ = false;
      return;
  }

  ekf_.F_ << 1, 0, dt_s, 0,
             0, 1, 0,    dt_s,
             0, 0, 1,    0,
             0, 0, 0,    1;

  double dt_s2 = dt_s * dt_s;
  double dt_s3 = dt_s2 * dt_s;
  double dt_s4 = dt_s2 * dt_s2;

  ekf_.Q_ <<
          dt_s4 * noise_ax_ / 4,    0,                      dt_s3 * noise_ax_ / 2,  0,
          0,                        dt_s4 * noise_ay_ / 4,  0,                      dt_s3 * noise_ay_ / 2,
          dt_s3 * noise_ax_ / 2,    0,                      dt_s2 * noise_ax_,      0,
          0,                        dt_s3 * noise_ay_ / 2,  0,                      dt_s2 * noise_ay_;

  ekf_.Predict();

  /**
   * Update:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //Radar updates
    try {
        Hj_ = tools.CalculateJacobian(ekf_.x_);
    } catch (...) {
        cout << "Error computing Jacobian" << endl; //Using the prior Jacobian in case of errors computing the above
    }
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
