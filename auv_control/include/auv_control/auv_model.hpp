#ifndef AUV_MODEL
#define AUV_MODEL

#include <ct/optcon/optcon.h>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "auv_control/nominal_thrust_solver.hpp"
#include "auv_navigation/auv_math_lib.hpp"
#include <cppad/cppad.hpp>
#include "math.h"

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

namespace AUVControl
{
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 6, 2> Matrix62d;

//typedef Matrix<double, 6, Dynamic> Matrix6Xd;
// Thruster matrices
typedef Eigen::Matrix<double, 5, Eigen::Dynamic> Matrix58d;
typedef Eigen::Matrix<double, 6, 8> Matrix68d;

// State Space Control Matrices
typedef Eigen::Matrix<double, 8, 8> Matrix8d;
typedef Eigen::Matrix<double, 12, 8> Matrix12x8d;
typedef Eigen::Matrix<double, 8, 12> Matrix8x12d;

typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
//typedef Matrix<CppAD::AD<double>, 1, Dynamic> ADRowVectorXd;

// AUV Model
// Contains information about an AUV's attributes: mass, volume inertia, drag, and thruster properties
// Used to compute any jacobians and state vectors required by TransEKF and LQR
class AUVModel
{
  typedef Eigen::Matrix<CppAD::AD<double>, 3, 3> ADMatrix3d;
  typedef Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, 1> ADVectorXd;
  typedef Eigen::Matrix<CppAD::AD<double>, 3, 1> ADVector3d;

private:
  double mass_, volume_, density_, Fg_, Fb_;
  int numThrusters_;
  Eigen::Matrix3d inertia_; // Inertia 3x3 matrix
  Matrix62d dragCoeffs_;
  Matrix58d thrusters_;
  Matrix68d thrustCoeffs_;
  Eigen::Vector3d CoB_;  // Center of buoyancy position relative to CoM
  Matrix12d A_; // Linearized system matrix
  Matrix12x8d B_; // Linearized control input matrix
  Matrix8x12d K_;

  // Ceres Problem
  ceres::Problem nominalThrustProblem;
  ceres::Solver::Options nominalThrustOptions;
  ceres::Solver::Summary nominalThrustSummary;
  double nominalForces[10];
  double quaternion_[4], uvw_[3], pqr_[3], inertialTransAccel_[3], pqrDot_[3];

public:
  // Calling this macro will fix alignment issues on members that are fixed-size Eigen objects
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr double GRAVITY = 9.80665;    // [m/s^2]
  static constexpr double WATER_DENSITY = 1000; // [kg/m^3]

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

  AUVModel(double mass, double volume, double fluid_density, const Eigen::Ref<const Eigen::Matrix3d> &inertia, const Eigen::Ref<const Eigen::Vector3d> &CoB,
           const Eigen::Ref<const Matrix62d> &dragCoeffs, const Eigen::Ref<const Matrix58d> &thrusters);

  void setThrustCoeffs();
  Vector6d getTotalThrustLoad(const Eigen::Ref<const Eigen::VectorXd> &thrusts);
  //Vector6d getWeightLoad(const Ref<const Vector3d> &attitude);

  void setLinearizedSystemMatrix(const Eigen::Ref<const Vector12d> &ref);
  void setLinearizedInputMatrix();
  Matrix12x8d getLinearizedInputMatrix();
  Matrix12d getLinearizedSystemMatrix();
};
} // namespace AUVControl

#endif