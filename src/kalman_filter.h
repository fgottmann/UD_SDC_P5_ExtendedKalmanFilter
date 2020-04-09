#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes discrete Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model x_kp1 = f and d_x_kp1 / d_x = F_ and the process noise Q
   */
  void Predict(const Eigen::VectorXd &f, const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q);

  /**
   * Updates the state by using  Kalman Filter equations
   * This is independent wether its a ekf or lkf as just the jacobians are switched
   * @param z The measurement error
   * @param H jacobian of measurement error
   * @param R measurement covariance
   */
  void Update(const Eigen::VectorXd &e, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);

  /**
  * Returns the current State
  */
  Eigen::VectorXd GetX();

  /**
  * Returns the current Covariance
  */
  Eigen::MatrixXd GetP();

private:
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;


};

#endif // KALMAN_FILTER_H_
