#ifndef MODEL_H_
#define MODEL_H_

#include <vector>
#include "Eigen/Dense"

class Model {
public:
  /**
  * Constructor.
  */
  Model();

  // Number of states
  const int Nx;

  /**
  * Destructor.
  */
  virtual ~Model();

  /**
  * A method to determine all values and derviatives neccessary to calculate the correction
  * due to lidar measurements.
  */
  int CalculateRadarMeasurement(const Eigen::VectorXd& x_state,const Eigen::VectorXd& z,
    Eigen::VectorXd& e, Eigen::MatrixXd* H, Eigen::MatrixXd* R);

    /**
    * A method to determine all values and derviatives neccessary to calculate the correction
    * due to lidar measurements.
    */
    int CalculateLidarMeasurement(const Eigen::VectorXd& x_state,const Eigen::VectorXd& z,
      Eigen::VectorXd& e, Eigen::MatrixXd* H, Eigen::MatrixXd* R);

      /**
      * A method to determine all values and derviatives neccessary to calculate the prediction
      * in the kalman filter.
      */
      int CalculateProcess(const Eigen::VectorXd& x_state,const double& dt,
        Eigen::VectorXd& f, Eigen::MatrixXd* F, Eigen::MatrixXd* Q);

      };


      #endif  // TOOLS_H_
