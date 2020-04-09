#include "FusionEKF.h"
#include <iostream>
#include <cmath>
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
  was_recently_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 6);
  H_laser_ << 1,0,0,0,0,0,
  0,1,0,0,0,0;

  //measurement covariance matrix - laser
  R_laser_ << 0.00225, 0,
  0, 0.00225;

  //measurement covariance matrix - radar
  R_radar_ << 0.025, 0, 0,
  0, 0.1,0,
  0, 0, 0.01;

  /**
  * Dummy initilization for kalman filter
  * real values are given in the intialization part of 'ProcessMeasurement'
  */
  VectorXd x_init = VectorXd::Zero(6);
  MatrixXd P_init = MatrixXd::Identity(x_init.rows(),x_init.rows())*1.0e-6;
  ekf_.Init(x_init,P_init);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

int FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
  * Initialization
  */
  // calc dt early so for initialization step no big dt's occure
  double dt = (measurement_pack.timestamp_ - previous_timestamp_)*1.0e-6;
  previous_timestamp_ = measurement_pack.timestamp_;

  if (!is_initialized_ || fabs(dt) >= 1) { // reinitialize after timeout of 1s
    /**
    * TODO: Initialize the state ekf_.x_ with the first measurement.
    * TODO: Create the covariance matrix.
    * You'll need to convert radar from polar to cartesian coordinates.
    */

    // first measurement
    cout << "EKF: " << endl;
    VectorXd x_init = VectorXd::Zero(ekf_.GetX().rows());
    MatrixXd P_init = MatrixXd::Identity(ekf_.GetX().rows(),ekf_.GetX().rows())*1.0e-8;
    P_init(0,0) = 1e-3; P_init(1,1) = 1e-3; P_init(3,3) = 1e0; P_init(2,2) = 1e1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.
      double x0 = measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]);
      double y0 = measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]);
      x_init(0) = x0;
      x_init(1) = y0;
      x_init(2) = sqrt(2.0)*measurement_pack.raw_measurements_[2];
      x_init(3) = sqrt(2.0)*measurement_pack.raw_measurements_[2];
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      x_init(0) = measurement_pack.raw_measurements_[0];
      x_init(1) = measurement_pack.raw_measurements_[1];
    }

    ekf_.Init(x_init,P_init);
    // done initializing, no need to predict or update
    is_initialized_ = true;
    was_recently_initialized_ = true;
    return 1;
  }
  // late initilization for the velocity
  if (was_recently_initialized_)
  {
    was_recently_initialized_ = false;
    if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      VectorXd x_init = ekf_.GetX();
      MatrixXd P_init = ekf_.GetP();
      x_init(2) = sqrt(2.0)*measurement_pack.raw_measurements_[2];
      x_init(3) = sqrt(2.0)*measurement_pack.raw_measurements_[2];
      ekf_.Init(x_init,P_init);
    }
  }

  /**
  * Prediction
  */
  MatrixXd Q = MatrixXd::Zero(ekf_.GetX().rows(),ekf_.GetX().rows());
  Q(4,4) = 5.0; Q(5,5) = 5.0;
  Q = Q*dt*dt; // sample time normalization for discrete kalmanfilter
  MatrixXd F = MatrixXd::Zero(ekf_.GetX().rows(),ekf_.GetX().rows());
  F(0,2) = 1;  F(1,3) = 1;  F(2,4) = 1;  F(3,5) = 1;
  F = MatrixXd::Identity(ekf_.GetX().rows(),ekf_.GetX().rows()) + F*dt + F*F*dt*dt/2.0 + F*F*F*dt*dt*dt/6.0; // taylor sequence
  VectorXd f = VectorXd::Zero(ekf_.GetX().rows());
  f = F*ekf_.GetX(); // linear case
  Q = F*Q*F.transpose();
  ekf_.Predict(f,F,Q);

  /**
  * Update
  */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    MatrixXd H = tools.CalculateJacobianRadarMeasurement(ekf_.GetX());
    VectorXd e = measurement_pack.raw_measurements_ - tools.CalculateRadarMeasurement(ekf_.GetX());
    e(1) = fmod(e(1) + M_PI_2,M_PI) - M_PI_2;
    MatrixXd R_radar = R_radar_;
    if (measurement_pack.raw_measurements_[0] < 5) R_radar(1,1) = 1.0e5; // disable angle for close objects
    if (measurement_pack.raw_measurements_[0] < 3) R_radar(2,2) = 1.0e5; // disable velocity for close objects
    ekf_.Update(e,H,R_radar);

  } else {
    // TODO: Laser updates
    VectorXd e = measurement_pack.raw_measurements_ - H_laser_ * ekf_.GetX(); // linear case
    ekf_.Update(e,H_laser_,R_laser_);
  }



  // print the output
  // cout << "x_ = " << ekf_.GetX() << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
  return 0;
}
