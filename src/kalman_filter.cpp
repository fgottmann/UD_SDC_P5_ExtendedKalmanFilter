#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
using std::cout;
using std::endl;

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in) {
  x_ = x_in;
  P_ = P_in;
}

void KalmanFilter::Predict(const Eigen::VectorXd &f, const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q) {
  /**
   * TODO: predict the state
   */
   x_ = f;
   P_ = F*P_*F.transpose() + Q;
}


void KalmanFilter::Update(const VectorXd &e, const MatrixXd &H, const MatrixXd &R) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   MatrixXd K = P_ * H.transpose()*( H*P_*H.transpose() + R).inverse();
   x_ = x_ + K*e;
   P_ = (MatrixXd::Identity(P_.rows(), P_.rows()) - K*H)*P_;
}

VectorXd KalmanFilter::GetX() {
  return x_;
}
MatrixXd KalmanFilter::GetP() {
  return P_;
}
