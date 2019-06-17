#ifndef KALMAN_FILTER
#define KALMAN_FILTER

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include <sstream>

namespace AUVNavigation
{
// Basic Kalman Filter
// If not initialized manually, then it will auto-initialize (set Xhat prediction to zero-vector).
class KalmanFilter
{
private:
  float m_, n_;          // m = # measurements, n = # states
  Eigen::VectorXf Xhat_; // State Vector
  Eigen::MatrixXf A_;    // State-transition Matrix
  Eigen::MatrixXf H_;    // Observation/Measurement Matrix
  Eigen::MatrixXf K_;    // Kalman Gain
  Eigen::MatrixXf P_;    // Error Covariance Matrix
  Eigen::MatrixXf Q_;    // Process Noise Covariance Matrix
  Eigen::MatrixXf R_;    // Measurement Noise Covariance Matrix
  Eigen::MatrixXf I_;    // Identity Matrix
  bool init_;

public:
  KalmanFilter(const Eigen::Ref<const Eigen::MatrixXf> &Ao, const Eigen::Ref<const Eigen::MatrixXf> &Ho,
               const Eigen::Ref<const Eigen::MatrixXf> &Qo, const Eigen::Ref<const Eigen::MatrixXf> &Ro);
  void init(const Eigen::Ref<const Eigen::VectorXf> &Xo);
  Eigen::VectorXf update(const Eigen::Ref<const Eigen::VectorXf> &Z);
  Eigen::VectorXf updateEKF(const Eigen::Ref<const Eigen::MatrixXf> &Anew, const Eigen::Ref<const Eigen::MatrixXf> &Hnew, const Eigen::Ref<const Eigen::MatrixXf> &Rnew,
                            const Eigen::Ref<const Eigen::VectorXf> &Xpredict, const Eigen::Ref<const Eigen::VectorXf> &Z);
  Eigen::VectorXf getXhat();
  Eigen::MatrixXf getErrorCovariance();
};
} // namespace AUVNavigation

#endif