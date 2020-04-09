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
