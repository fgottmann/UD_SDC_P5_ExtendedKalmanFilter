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
    * TODO: Calculate the RMSE here.
    */

    VectorXd RMSE;
    if (estimations.size() > 0 && estimations.size() == ground_truth.size())
    {
      RMSE = VectorXd::Zero(estimations[0].rows());
      for (int ii = 0; ii < estimations.size();ii++)
      {
        VectorXd diff = estimations[ii]-ground_truth[ii];
        for (int jj = 0; jj < estimations[0].rows(); jj++)
          RMSE(jj) += diff(jj)*diff(jj);
      }


      for (int ii = 0; ii < estimations[0].rows(); ii++)
        RMSE(ii) = sqrt(RMSE(ii)/double(estimations.size()));


    }


    return RMSE;
  }

  MatrixXd Tools::CalculateJacobianRadarMeasurement(const VectorXd& x_state) {
    /**
    * TODO:
    * Calculate a Jacobian here.
    */
    MatrixXd ret = MatrixXd::Zero(3,x_state.rows());
    double divider = 1.0/std::max(1e-3,sqrt(x_state(0)*x_state(0) + x_state(1)*x_state(1)));
    ret(0,0) = x_state(0)*divider;
    ret(0,1) = x_state(1)*divider;
    ret(1,0) = -x_state(1)*divider*divider;
    ret(1,1) = x_state(0)*divider*divider;
    ret(2,0) = x_state(1)*(x_state(2)*x_state(1) - x_state(3)*x_state(0))*divider*divider*divider;
    ret(2,1) = x_state(0)*(x_state(3)*x_state(0) - x_state(2)*x_state(1))*divider*divider*divider;
    ret(2,2) = x_state(0)*divider;
    ret(2,3) = x_state(1)*divider;
    return ret;

  }

  VectorXd Tools::CalculateRadarMeasurement(const Eigen::VectorXd& x_state)
  {
    MatrixXd ret = VectorXd::Zero(3);
    ret(0) = sqrt(x_state(0)*x_state(0) + x_state(1)*x_state(1));
    ret(1) = atan(x_state(1)/x_state(0));
    ret(2) = (x_state(0)*x_state(2) + x_state(1)*x_state(3))/std::max(1e-3,ret(0));
    return ret;
  }
