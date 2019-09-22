#ifndef AUV_MODEL
#define AUV_MODEL

#include <ct/optcon/optcon.h>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "auv_control/nominal_thrust_solver.hpp"
#include "auv_core/math_lib.hpp"
#include "auv_core/constants.hpp"
#include <cppad/cppad.hpp>
#include "math.h"
#include <algorithm>

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

namespace auv_control
{
typedef Eigen::Matrix<double, 6, 2> Matrix62d;

// Thruster matrices
typedef Eigen::Matrix<double, 5, 8> Matrix58d;
typedef Eigen::Matrix<double, 6, 8> Matrix68d;

// State Space Control Matrices
typedef Eigen::Matrix<double, 18, 18> Matrix18d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 8, 8> Matrix8d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 18, 8> Matrix18x8d;
typedef Eigen::Matrix<double, 12, 8> Matrix12x8d;
typedef Eigen::Matrix<double, 8, 18> Matrix8x18d;
typedef Eigen::Matrix<double, 8, 12> Matrix8x12d;

typedef Eigen::Matrix<double, 18, 1> Vector18d;
typedef Eigen::Matrix<double, 13, 1> Vector13d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// AUV Model
// Contains information about an AUV's attributes: mass, volume inertia, drag, and thruster properties
// Used to compute any jacobians and state vectors required by TransEKF and LQR
class AUVModel
{
   typedef Eigen::Matrix<CppAD::AD<double>, 3, 3> ADMatrix3d;
   typedef Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, 1> ADVectorXd;
   typedef Eigen::Matrix<CppAD::AD<double>, 3, 1> ADVector3d;

private:
   double mass_, Fg_, Fb_;
   int numThrusters_, maxThrusters_;
   Eigen::Matrix3d inertia_; // Inertia 3x3 matrix
   Matrix62d dragCoeffs_;
   Matrix58d thrusterData_;
   Matrix68d thrustCoeffs_;
   Eigen::Vector3d CoB_; // Center of buoyancy position relative to CoM

   // LQR Matrices
   Matrix12d A_;      // (Linearized) system matrix
   Matrix18d augA_;   // (Linearized) augmented system matrix
   Matrix12x8d B_;    // Control input matrix
   Matrix18x8d augB_; // Augmented control input matrix
   Matrix8x12d K_;    // Gain matrix
   Matrix8x18d augK_; // Augmented gain matrix
   Matrix12d Q_;      // State cost matrix
   Matrix18d augQ_;   // Augmented state cost matrix
   Matrix8d R_;       // Input cost matrix

   // LQR Variables
   Vector8d totalThrust_;
   Vector8d lqrThrust_;
   Vector12d error_;
   Vector18d augError_;
   Eigen::Vector3d positionIntegralError_;
   Eigen::Quaterniond qState_, qRef_, qError_, qIntegralError_;

   // Ceres Problem
   ceres::Problem problemNominalThrust;
   ceres::Solver::Options optionsNominalThrust;
   ceres::Solver::Summary summaryNominalThrust;
   double nominalThrust_[8];
   double quaternion_[4], uvw_[3], pqr_[3], inertialTransAccel_[3], pqrDot_[3];

   // LQR Setup
   // The ct_optcon LQR solver requires these sizes be defined at compile time
   static const size_t state_dim = 12;
   static const size_t state_dim_aug = 18;
   static const size_t control_dim = 8;
   ct::optcon::LQR<state_dim, control_dim> lqrSolver_;
   ct::optcon::LQR<state_dim_aug, control_dim> lqrAugSolver_;
   bool initLQR_, enableLQRIntegral_;

   void setThrustCoeffs();
   void setLinearizedSystemMatrix(const Eigen::Ref<const Vector13d> &ref);
   void setLinearizedInputMatrix();

public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   AUVModel(double Fg, double Fb,
            const Eigen::Ref<const Eigen::Vector3d> &CoB,
            const Eigen::Ref<const Eigen::Matrix3d> &inertia,
            const Eigen::Ref<const Matrix62d> &dragCoeffs,
            const Eigen::Ref<const Matrix58d> &thrusterData,
            int numThrusters);

   void setLQRCostMatrices(const Eigen::Ref<const Matrix12d> &Q, const Eigen::Ref<const Matrix8d> &R);
   void setLQRIntegralCostMatrices(const Eigen::Ref<const Matrix18d> &augQ, const Eigen::Ref<const Matrix8d> &R);

   Vector6d getTotalThrustLoad(const Eigen::Ref<const Vector8d> &thrusts);
   Vector8d computeLQRThrust(const Eigen::Ref<const Vector13d> &state,
                             const Eigen::Ref<const Vector13d> &ref,
                             const Eigen::Ref<const Vector6d> &accel);
};
} // namespace auv_control

#endif