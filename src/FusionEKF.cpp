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

    /**
    * Dummy initilization for kalman filter
    * real values are given in the intialization part of 'ProcessMeasurement'
    */

    VectorXd x_init = VectorXd::Zero(model_.Nx);
    MatrixXd P_init = MatrixXd::Identity(model_.Nx,model_.Nx)*1.0e-6;
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
        // first measurement
      cout << "EKF: " << endl;
      VectorXd x_init = VectorXd::Zero(model_.Nx);
      MatrixXd P_init = MatrixXd::Identity(model_.Nx,model_.Nx)*1.0e-8;
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
    // late initilization for the velocity using radar if it's now available
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
    VectorXd f; MatrixXd F; MatrixXd Q;
    model_.CalculateProcess(ekf_.GetX(),dt,f, &F, &Q);
    ekf_.Predict(f,F,Q);

    /**
    * Update
    */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      VectorXd e; MatrixXd H; MatrixXd R;
      model_.CalculateRadarMeasurement(ekf_.GetX(),measurement_pack.raw_measurements_,e, &H, &R);
      ekf_.Update(e,H,R);

    } else {
      // TODO: Laser updates
        VectorXd e; MatrixXd H; MatrixXd R;
        model_.CalculateLidarMeasurement(ekf_.GetX(),measurement_pack.raw_measurements_,e, &H, &R);
      ekf_.Update(e,H,R);
    }



    // print the output
    // cout << "x_ = " << ekf_.GetX() << endl;
    // cout << "P_ = " << ekf_.P_ << endl;
    return 0;
  }
