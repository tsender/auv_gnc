#ifndef KALMAN_FILTER
#define KALMAN_FILTER

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include <sstream>

using namespace Eigen;

namespace AUV_GNC
{
// Basic Kalman Filter
// If not initialized manually, then it will auto-initialize (set Xhat prediction to zero-vector).
class KalmanFilter
{
  private:
    float m_, n_;    // m = # measurements, n = # states
    VectorXf Xhat_; // State Vector
    MatrixXf A_;    // State-transition Matrix
    MatrixXf H_;    // Observation/Measurement Matrix
    MatrixXf K_;    // Kalman Gain
    MatrixXf P_;    // Error Covariance Matrix
    MatrixXf Q_;    // Process Noise Covariance Matrix
    MatrixXf R_;    // Measurement Noise Covariance Matrix
    MatrixXf I_;    // Identity Matrix
    bool init_;

  public:
    KalmanFilter(const Ref<const MatrixXf> &Ao, const Ref<const MatrixXf> &Ho,
                 const Ref<const MatrixXf> &Qo, const Ref<const MatrixXf> &Ro);
    void init(const Ref<const VectorXf> &Xo);
    VectorXf update(const Ref<const VectorXf> &Z);
    VectorXf updateEKF(const Ref<const MatrixXf> &Anew, const Ref<const MatrixXf> &Hnew, const Ref<const MatrixXf> &Rnew,
                       const Ref<const VectorXf> &Xpredict, const Ref<const VectorXf> &Z);
    VectorXf getXhat();
    MatrixXf getErrorCovariance();
};
}

#endif