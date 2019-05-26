#ifndef EKF_TRANSLATION
#define EKF_TRANSLATION

#include "auv_gnc/kalman_filter.h"
#include "auv_gnc/auv_math_lib.h"
#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;
using namespace std;
typedef Matrix<int, 1, Dynamic> RowXi;
typedef Matrix<float, 3, 2> Matrix32f;
typedef Matrix<float, 9, 9> Matrix9f;

// Translational Extended Kalman Filter
// This class is designed to estimate a vehicle's position as expressed in the I-frame,
// and the vehicle's linear velocity and acceleration as expressed in the B-frame.
// Since the sensory input come from INERTIAL sensors, state predictions are ALSO inertial.
class TransEKF
{
private:
  KalmanFilter *EKF;
  Vector9f Xhat;
  Marix3i fullMsmtMask;
  Matrix3f Rpos, Rvel, Raccel;
  Matrix9Xf Q;
  bool init;
  int n; // Size of A matrix (nxn = 9x9)

public:
  TransEKF(const Ref<const Matrix3i> &sensorMaskIn, const Ref<const MatrixXf> &RposIn, const Ref<const MatrixXf> &RvelIn,
           const Ref<const MatrixXf> &RaccelIn, const Ref<const Matrix9f> &Qin);
  void Init(const Ref<const VectorXf> &Xo);
  VectorXf Update(float dt, const Ref<const Vector3f> &attitude, const Ref<const Vector3i> &sensorMask, const Ref<const Matrix3f> &Zmat)
};

#endif