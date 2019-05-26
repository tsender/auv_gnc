#ifndef AUV_MODEL
#define AUV_MODEL

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "auv_gnc/auv_math_lib.h"
#include <cppad/cppad.hpp>

using namespace Eigen;
using namespace std;
using namespace AUVMathLib;
using CppAD::AD;

typedef Matrix<float, 12, 12> Matrix12f;
typedef Matrix<float, 6, 2> Matrix62f;
typedef Matrix<float, 3, 2> Matrix32f;
typedef Matrix<float, 9, 9> Matrix9f;
typedef Matrix<float, 6, Dynamic> Matrix6xf;
typedef Matrix<float, 5, Dynamic> Matrix5xf;

typedef Matrix<float, 12, 1> Vector12f;
typedef Matrix<float, 9, 1> Vector9f;
typedef Matrix<float, 6, 1> Vector6f;
typedef Matrix<float, 5, 1> Vector5f;

typedef Matrix< CppAD::AD<double> , Dynamic, Dynamic > ADMatrixXd;
typedef Matrix< CppAD::AD<double> , Dynamic, 1 > ADVectorXd;
typedef Matrix< CppAD::AD<double> , 1, Dynamic > ADRowVectorXd;

// Constants used for Auto Diff
// Indeces for state vector
#define _xI 0    // Inertial X-pos, expressed in I-frame
#define _yI 1    // Inertial Y-pos, expressed in I-frame
#define _zI 2    // Inertial Z-pos, expressed in I-frame
#define _psi 3   // Yaw
#define _theta 4 // Pitch
#define _phi 5   // Roll
#define _U 6     // Inertial X velocity , expressed in B-frame
#define _V 7     // Inertial Y velocity , expressed in B-frame
#define _W 8     // Inertial Z velocity , expressed in B-frame
#define _P 9     // Inertial X angular velocity , expressed in B-frame
#define _Q 10    // Inertial Y angular velocity , expressed in B-frame
#define _R 11    // Inertial Z angular velocity , expressed in B-frame

// Add new typedefs to Eigen namespace so we can use CppAD with it
namespace Eigen {
template<typename X, typename BinOp>
struct ScalarBinaryOpTraits<CppAD::AD<X>,X,BinOp>
{
  typedef CppAD::AD<X> ReturnType;
};

template<typename X, typename BinOp>
struct ScalarBinaryOpTraits<X,CppAD::AD<X>,BinOp>
{
  typedef CppAD::AD<X> ReturnType;
};
} // namespace Eigen

// AUV Model
// Contains information about an AUV's attributes: mass, volume inertia, drag, and thruster properties
// Used to compute any jacobians and state vectors required by TransEKF and LQR
class AUVModel
{
private:
  float mass, vol, rho, Fg, Fb;
  float Ixx, Ixy, Ixz, Iyy, Iyz, Izz;
  int numThrusters;
  Matrix3f inertia; // Inertia 3x3 matrix
  Matrix32f dragCoeffs;
  Matrix5xf thrusters;
  Matrix6xf thrustCoeffs;
  Vector3f CoB; // Center of buoyancy position relative to CoM

public:
  // Calling this macro will fix alignment issues on members that are fixed-size Eigen objects
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const static float PI = 3.141592653;
  const static float GRAVITY = 9.80665;    // [m/s^2]
  const static float WATER_DENSITY = 1000; // [kg/m^3]

  AUVModel(float m, float V, float fluid_rho, const Ref<const Matrix3f> &Inertia, const Ref<const Vector3f> &cob,
           const Ref<const Matrix62f> &drag, vector<Vector5f> &auv_thrusters);

  void SetThrustCoeffs();
  Vector6f GetTotalThrustLoad(const Ref<const VectorXf> &thrusts);
  Vector6f GetWeightLoad(float phi, float theta);

  void SetInitialLQRJacobianA();
  Matrix12f GetStateJacobian(const Ref<const Vector12f> &ref);
};

#endif