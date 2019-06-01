#ifndef AUV_MODEL
#define AUV_MODEL

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "auv_control/nominal_thrust_solver.hpp"
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
typedef Matrix<double, 12, 12> Matrix12d;
typedef Matrix<double, 12, Dynamic> Matrix12Xd;
typedef Matrix<double, 6, 2> Matrix62d;
typedef Matrix<double, 6, Dynamic> Matrix6Xd;
typedef Matrix<double, 6, 10> Matrix610d;
typedef Matrix<double, 5, Dynamic> Matrix5Xd;

typedef Matrix<double, 12, 1> Vector12d;
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 5, 1> Vector5d;
typedef Matrix<double, 4, 1> Vector4d;

typedef Matrix<CppAD::AD<double>, 3, 3> ADMatrix3d;
typedef Matrix<CppAD::AD<double>, Dynamic, 1> ADVectorXd;
typedef Matrix<CppAD::AD<double>, 3, 1> ADVector3d;
//typedef Matrix<CppAD::AD<double>, 1, Dynamic> ADRowVectorXd;

// AUV Model
// Contains information about an AUV's attributes: mass, volume inertia, drag, and thruster properties
// Used to compute any jacobians and state vectors required by TransEKF and LQR
class AUVModel
{
private:
  double mass_, volume_, density_, Fg_, Fb_;
  int numThrusters_;
  Matrix3d inertia_; // Inertia 3x3 matrix
  Matrix62d dragCoeffs_;
  Matrix5Xd thrusters_;
  Matrix6Xd thrustCoeffs_;
  Vector3d CoB_; // Center of buoyancy position relative to CoM
  Matrix12Xd B_; // Linearized control input matrix

public:
  // Calling this macro will fix alignment issues on members that are fixed-size Eigen objects
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const double PI = 3.141592653;
  static const double GRAVITY = 9.80665;    // [m/s^2]
  static const double WATER_DENSITY = 1000; // [kg/m^3]

  // Useful indeces
  static const int xI_ = 0; // Inertial X-pos, expressed in I-frame
  static const int yI_ = 1; // Inertial Y-pos, expressed in I-frame
  static const int zI_ = 2; // Inertial Z-pos, expressed in I-frame
  static const int U_ = 3;  // Inertial X velocity , expressed in B-frame
  static const int V_ = 4;  // Inertial Y velocity , expressed in B-frame
  static const int W_ = 5;  // Inertial Z velocity , expressed in B-frame
  static const int q1_ = 6; // Quaternion (I->B Frame) i-component
  static const int q2_ = 7; // Quaternion (I->B Frame) j-component
  static const int q3_ = 8; // Quaternion (I->B Frame) k-component
  static const int P_ = 9;  // Inertial X angular velocity , expressed in B-frame
  static const int Q_ = 10; // Inertial Y angular velocity , expressed in B-frame
  static const int R_ = 11; // Inertial Z angular velocity , expressed in B-frame

  AUVModel(double mass, double volume, double fluid_density, const Ref<const Matrix3d> &inertia, const Ref<const Vector3d> &CoB,
           const Ref<const Matrix62d> &dragCoeffs, const Ref<const Matrix5Xd> &thrusters);

  void setThrustCoeffs();
  Vector6d getTotalThrustLoad(const Ref<const VectorXd> &thrusts);
  //Vector6d getWeightLoad(const Ref<const Vector3d> &attitude);

  Matrix12d getLinearizedSystemMatrix(const Ref<const Vector12d> &ref);
  void setLinearizedInputMatrix();
  Matrix12Xd getLinearizedInputMatrix();
};
} // namespace AUV_GNC

#endif