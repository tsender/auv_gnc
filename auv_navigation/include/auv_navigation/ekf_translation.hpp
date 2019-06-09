#ifndef EKF_TRANSLATION
#define EKF_TRANSLATION

#include "auv_navigation/kalman_filter.hpp"
#include "auv_navigation/auv_math_lib.hpp"
#include "eigen3/Eigen/Dense"
#include "math.h"
#include <sstream>

using namespace Eigen;

namespace AUV_GNC
{
typedef Matrix<int, 1, Dynamic> RowXi;
typedef Matrix<float, 9, 1> Vector9f;
typedef Matrix<float, 9, 9> Matrix9f;

// Translational Extended Kalman Filter
// This class is designed to estimate a vehicle's position as expressed in the I-frame,
// and the vehicle's linear velocity and acceleration as expressed in the B-frame.
// Since the sensory input come from INERTIAL sensors, state predictions are ALSO inertial.
class EKFTranslation
{
private:
  KalmanFilter *ekf_;
  Vector9f Xhat_;
  Matrix3i fullMsmtMask_;
  Matrix3f Rpos_, Rvel_, Raccel_;
  Matrix9f Q_;
  bool init_;
  int n_; // Size of A matrix (nxn = 9x9)

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  EKFTranslation(const Ref<const Matrix3i> &sensorMaskIn, const Ref<const MatrixXf> &RposIn, const Ref<const MatrixXf> &RvelIn,
           const Ref<const MatrixXf> &RaccelIn, const Ref<const Matrix9f> &Qin);
  void init(const Ref<const VectorXf> &Xo);
  VectorXf update(float dt, const Ref<const Vector3f> &attitude, const Ref<const Vector3i> &sensorMask, const Ref<const Matrix3f> &Zmat);
};
}

#endif