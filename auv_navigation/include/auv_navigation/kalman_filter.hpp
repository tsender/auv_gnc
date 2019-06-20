#ifndef KALMAN_FILTER
#define KALMAN_FILTER

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include <sstream>

namespace auv_navigation
{
// Basic Kalman Filter
// If not initialized manually, then it will auto-initialize (set Xhat prediction to zero-vector).
class KalmanFilter
{
private:
  float m_, n_;          // m = # measurements, n = # states
  Eigen::VectorXd Xhat_; // State Vector
  Eigen::MatrixXd A_;    // State-transition Matrix
  Eigen::MatrixXd H_;    // Observation/Measurement Matrix
  Eigen::MatrixXd K_;    // Kalman Gain
  Eigen::MatrixXd P_;    // Error Covariance Matrix
  Eigen::MatrixXd Q_;    // Process Noise Covariance Matrix
  Eigen::MatrixXd R_;    // Measurement Noise Covariance Matrix
  Eigen::MatrixXd I_;    // Identity Matrix
  bool init_;

public:
  KalmanFilter(const Eigen::Ref<const Eigen::MatrixXd> &Ao,
               const Eigen::Ref<const Eigen::MatrixXd> &Ho,
               const Eigen::Ref<const Eigen::MatrixXd> &Qo,
               const Eigen::Ref<const Eigen::MatrixXd> &Ro);
  void init(const Eigen::Ref<const Eigen::VectorXd> &Xo);
  Eigen::VectorXd update(const Eigen::Ref<const Eigen::VectorXd> &Z);
  Eigen::VectorXd updateEKF(const Eigen::Ref<const Eigen::MatrixXd> &Anew,
                            const Eigen::Ref<const Eigen::MatrixXd> &Hnew, 
                            const Eigen::Ref<const Eigen::MatrixXd> &Rnew,
                            const Eigen::Ref<const Eigen::VectorXd> &Xpredict, 
                            const Eigen::Ref<const Eigen::VectorXd> &Z);
  Eigen::VectorXd getXhat();
  Eigen::MatrixXd getErrorCovariance();
};
} // namespace auv_navigation

#endif