#ifndef KALMAN_FILTER
#define KALMAN_FILTER

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "sstream"

using namespace Eigen;

// Basic Kalman Filter
// If not initialized manually, then it will auto-initialize (set Xhat prediction to zero-vector).
class KalmanFilter
{
  private:
    float m, n;    // m = # measurements, n = # states
    VectorXf Xhat; // State Vector
    MatrixXf A;    // State-transition Matrix
    MatrixXf H;    // Observation/Measurement Matrix
    MatrixXf K;    // Kalman Gain
    MatrixXf P;    // Error Covariance Matrix
    MatrixXf Q;    // Process Noise Covariance Matrix
    MatrixXf R;    // Measurement Noise Covariance Matrix
    MatrixXf I;    // Identity Matrix
    bool init;

  public:
    KalmanFilter(const Ref<const MatrixXf> &Ao, const Ref<const MatrixXf> &Ho,
                 const Ref<const MatrixXf> &Qo, const Ref<const MatrixXf> &Ro);
    void Init(const Ref<const VectorXf> &Xo);
    VectorXf Update(const Ref<const VectorXf> &Z);
    VectorXf EKFUpdate(const Ref<const MatrixXf> &Anew, const Ref<const MatrixXf> &Hnew, const Ref<const MatrixXf> &Rnew
                       const Ref<const VectorXf> &Xpredict, const Ref<const VectorXf> &Z);
    VectorXf GetXhat();
    MatrixXf GetErrorCovariance();
};

#endif