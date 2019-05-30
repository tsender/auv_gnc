#ifndef AUV_MODEL
#define AUV_MODEL

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "auv_navigation/auv_math_lib.hpp"
#include <cppad/cppad.hpp>

using namespace Eigen;
//using namespace std;
//namespace AUVMath =  AUV_GNC::AUVMathLib;
//using namespace AUVMath;
//using CppAD::AD;

// Constants used for Auto Diff
// Indeces for state vector

// Add new typedefs to Eigen namespace so we can use CppAD with it
namespace Eigen
{
template <typename X, typename BinOp>
struct ScalarBinaryOpTraits<CppAD::AD<X>, X, BinOp>
{
  typedef CppAD::AD<X> ReturnType;
};

template <typename X, typename BinOp>
struct ScalarBinaryOpTraits<X, CppAD::AD<X>, BinOp>
{
  typedef CppAD::AD<X> ReturnType;
};
} // namespace Eigen

namespace AUV_GNC
{
typedef Matrix<float, 12, 12> Matrix12f;
typedef Matrix<float, 6, 2> Matrix62f;
typedef Matrix<float, 3, 2> Matrix32f;
typedef Matrix<float, 9, 9> Matrix9f;
typedef Matrix<float, 6, Dynamic> Matrix6Xf;
typedef Matrix<float, 5, Dynamic> Matrix5Xf;

typedef Matrix<float, 12, 1> Vector12f;
typedef Matrix<float, 9, 1> Vector9f;
typedef Matrix<float, 6, 1> Vector6f;
typedef Matrix<float, 5, 1> Vector5f;

typedef Matrix<CppAD::AD<double>, Dynamic, Dynamic> ADMatrixXd;
typedef Matrix<CppAD::AD<double>, Dynamic, 1> ADVectorXd;
typedef Matrix<CppAD::AD<double>, 1, Dynamic> ADRowVectorXd;

// AUV Model
// Contains information about an AUV's attributes: mass, volume inertia, drag, and thruster properties
// Used to compute any jacobians and state vectors required by TransEKF and LQR
class AUVModel
{
private:
  float mass_, volume_, density_, Fg_, Fb_;
  float Ixx_, Ixy_, Ixz_, Iyy_, Iyz_, Izz_;
  int numThrusters_;
  Matrix3d inertia_; // Inertia 3x3 matrix
  Matrix62f dragCoeffs_;
  Matrix5Xf thrusters_;
  Matrix6Xf thrustCoeffs_;
  Vector3f CoB_; // Center of buoyancy position relative to CoM

public:
  // Calling this macro will fix alignment issues on members that are fixed-size Eigen objects
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const float PI = 3.141592653;
  static const float GRAVITY = 9.80665;    // [m/s^2]
  static const float WATER_DENSITY = 1000; // [kg/m^3]

  // Useful indeces
  static const int xI_ = 0;    // Inertial X-pos, expressed in I-frame
  static const int yI_ = 1;    // Inertial Y-pos, expressed in I-frame
  static const int zI_ = 2;    // Inertial Z-pos, expressed in I-frame
  static const int phi_ = 3;   // Roll
  static const int theta_ = 4; // Pitch
  static const int psi_ = 5;   // Yaw
  static const int U_ = 6;     // Inertial X velocity , expressed in B-frame
  static const int V_ = 7;     // Inertial Y velocity , expressed in B-frame
  static const int W_ = 8;     // Inertial Z velocity , expressed in B-frame
  static const int P_ = 9;     // Inertial X angular velocity , expressed in B-frame
  static const int Q_ = 10;    // Inertial Y angular velocity , expressed in B-frame
  static const int R_ = 11;    // Inertial Z angular velocity , expressed in B-frame

  AUVModel(float mass, float volume, float fluid_density, const Ref<const Matrix3d> &inertia, const Ref<const Vector3f> &cob,
                  const Ref<const Matrix62f> &drag, const Ref<const Matrix5Xf> &thrusters);

  void setThrustCoeffs();
  Vector6f getTotalThrustLoad(const Ref<const VectorXf> &thrusts);
  Vector6f getWeightLoad(const Ref<const Vector3f> &attitude);

  void setInitialLQRJacobianA();
  Matrix12f getStateJacobian(const Ref<const Vector12f> &ref);
};
} // namespace AUV_GNC

#endif