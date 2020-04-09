#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd CalculateJacobianRadarMeasurement(const Eigen::VectorXd& x_state);

  /**
   * A helper method to get the comparison value to the radar measurements from the KF.
   */
  Eigen::VectorXd CalculateRadarMeasurement(const Eigen::VectorXd& x_state);

};

#endif  // TOOLS_H_
