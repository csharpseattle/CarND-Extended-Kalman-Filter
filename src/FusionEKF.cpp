#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define SECONDS_IN_ONE_DAY 86400

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);

  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  ekf_.x_ = VectorXd(4);

  // The initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // The initial state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1,  0,  0,    0,
             0,  1,  0,    0,
             0,  0,  1000, 0,
             0,  0,  0,    1000;

  //
  // Initialization of Q, Process covariance matrix
  //
  ekf_.Q_ = MatrixXd::Zero(4, 4);

  //
  // Using 9 per Udacity for ax and ay
  //
  noise_ax = 9;
  noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}


void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  //
  // Calculate elapsed time
  //
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  //
  // Initialization.
  // If the initialization flag is not set or the elapsed
  // time is less than zero or longer than one day we initialize
  //
  if (!is_initialized_ || dt > SECONDS_IN_ONE_DAY || dt < 0)
  {
    //
    // Initialize the state ekf_.x_ with the first measurement.
    //
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      //
      // radar measurements need to be converted from polar coordinates
      // to cartesian.
      //
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];

      float px = rho * cos(phi);
      float py = rho * sin(phi);
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);

      ekf_.x_ << px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      //
      // lidar measurements are already cartesian.
      //
      double px = measurement_pack.raw_measurements_[0];
      double py = measurement_pack.raw_measurements_[1];
      ekf_.x_ << px, py, 0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //
  // Update the state transition matrix F
  // with the new elapsed time.
  //
  ekf_.F_(0, 2) = ekf_.F_(1, 3) = dt;

  //
  // The process noise covariance matrix Q
  //
  double dt2   = dt * dt;
  double dt3   = dt2 * dt;
  double dt4   = dt3 * dt;
  double dt4_4 = dt4 / 4;
  double dt3_2 = dt3 / 2;

  ekf_.Q_ << (dt4_4 * noise_ax), 0, (dt3_2 * noise_ax), 0,
             0, (dt4_4 * noise_ay), 0, (dt3_2 * noise_ay),
             (dt3_2 * noise_ax), 0, (dt2 * noise_ax), 0,
             0, (dt3_2 * noise_ay), 0, (dt2 * noise_ay);

  //
  // Prediction
  //
  ekf_.Predict();



  /*****************************************************************************
   *  Update
   ****************************************************************************/


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    //
    // Radar updates
    //
    ekf_.R_ = R_radar_;

    //
    // Since Radar is a non-linear measurement function
    // we use a linear approximation of h.  To do this we
    // use a multi-variate Taylor Series expansion.
    // First we will need to calculate the Jacobian matrix.
    //
    MatrixXd Hj = tools.CalculateJacobian(ekf_.x_);


    //
    // Check the Jacobian for validity.
    //
    if (Hj(0) != 0)
    {
      //
      // We have a valid Jacobian. Update the kalman filter
      // with it and call the Extended Update.
      //
      ekf_.H_ = Hj;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    else
    {
      //
      // possible division by zero in Jacobian.
      //
      std::cout << "Invalid Jacobian, skipping update\n";
    }
  }
  else
  {
    //
    // Laser updates
    //
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
