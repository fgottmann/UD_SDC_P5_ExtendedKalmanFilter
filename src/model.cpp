#include "model.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


Model::~Model() {}

Model::Model():Nx(6) {}

int  Model::CalculateRadarMeasurement(const Eigen::VectorXd& x_state,const Eigen::VectorXd& z,
  Eigen::VectorXd& e, Eigen::MatrixXd* H, Eigen::MatrixXd* R) {

    // get measurement equivalent of the model
    MatrixXd h = VectorXd::Zero(3);
    h(0) = sqrt(x_state(0)*x_state(0) + x_state(1)*x_state(1));
    h(1) = atan(x_state(1)/x_state(0));
    h(2) = (x_state(0)*x_state(2) + x_state(1)*x_state(3))/std::max(1e-3,h(0));

    e = z - h;

    // calculate the corresponding jacobian
    if (NULL != H)
    {
      H[0] = MatrixXd::Zero(3,x_state.rows());
      double divider = 1.0/std::max(1e-3,sqrt(x_state(0)*x_state(0) + x_state(1)*x_state(1)));
      H[0](0,0) = x_state(0)*divider;
      H[0](0,1) = x_state(1)*divider;
      H[0](1,0) = -x_state(1)*divider*divider;
      H[0](1,1) = x_state(0)*divider*divider;
      H[0](2,0) = x_state(1)*(x_state(2)*x_state(1) - x_state(3)*x_state(0))*divider*divider*divider;
      H[0](2,1) = x_state(0)*(x_state(3)*x_state(0) - x_state(2)*x_state(1))*divider*divider*divider;
      H[0](2,2) = x_state(0)*divider;
      H[0](2,3) = x_state(1)*divider;
    }

    // set the covariance of this particular measurement
    if (NULL != R)
    {
      R[0] = MatrixXd(3, 3);
      R[0] << 0.025, 0, 0,
      0, 0.1,0,
      0, 0, 0.01;
      if (z[0] < 5) R[0](1,1) = 1.0e5; // disable angle for close objects
      if (z[0] < 3) R[0](2,2) = 1.0e5; // disable velocity for close objects

    }

    return 0;
  }


  int  Model::CalculateLidarMeasurement(const Eigen::VectorXd& x_state,const Eigen::VectorXd& z,
    Eigen::VectorXd& e, Eigen::MatrixXd* H, Eigen::MatrixXd* R) {

      // get measurement equivalent of the model
      MatrixXd h = VectorXd::Zero(2);

      h(0) = x_state(0);
      h(1) = x_state(1);

      e = z - h;

      // calculate the corresponding jacobian
      if (NULL != H)
      {
        H[0] = MatrixXd(2, 6);
        H[0] << 1,0,0,0,0,0,
        0,1,0,0,0,0;
      }

      // set the covariance of this particular measurement
      if (NULL != R)
      {
        R[0] = MatrixXd(2, 2);
        R[0] << 0.00225, 0,
        0, 0.00225;
      }

      return 0;
    }


    int Model::CalculateProcess(const Eigen::VectorXd& x_state,const double& dt,
      Eigen::VectorXd& f, Eigen::MatrixXd* F, Eigen::MatrixXd* Q) {

        // get state transition of the process (here everything is pretty linear)
        MatrixXd F_ = MatrixXd::Zero(Nx,Nx);
        F_(0,2) = 1;  F_(1,3) = 1;  F_(2,4) = 1;  F_(3,5) = 1;
        F_ = MatrixXd::Identity(Nx,Nx) +
        F_*dt + F_*F_*dt*dt/2.0 + F_*F_*F_*dt*dt*dt/6.0; // taylor sequence
        f = F_*x_state; // linear case

        // calculate the corresponding jacobian
        if (NULL != F)
        {
          F[0] = F_;
        }

        // set the covariance of this particular measurement
        if (NULL != Q)
        {
          Q[0] = MatrixXd::Zero(Nx,Nx);
          Q[0](4,4) = 5.0; Q[0](5,5) = 5.0;
          Q[0] = Q[0]*dt*dt; // sample time normalization for discrete kalmanfilter
          Q[0] = F_*Q[0]*F_.transpose();
        }

        return 0;
      }
