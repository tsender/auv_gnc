#ifndef AUV_LQR
#define AUV_LQR

#include "auv_core/auv_core_headers.hpp"
#include "auv_control/nominal_thrust_solver.hpp"

#include <ct/optcon/optcon.h>
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
// AUV Model
// Contains information about an AUV's attributes: mass, volume inertia, drag, and thruster properties
// Used to compute any jacobians and state vectors required by TransEKF and LQR
class AUVLQR
{
   typedef Eigen::Matrix<CppAD::AD<double>, 3, 3> ADMatrix3d;
   typedef Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, 1> ADVectorXd;
   typedef Eigen::Matrix<CppAD::AD<double>, 3, 1> ADVector3d;

private:
   auv_core::auvParameters *auvParams_;
   auv_core::Matrix68d thrustCoeffs_;
   int numThrusters_;
   int maxThrusters_;
   double dt_;

   // LQR Matrices
   auv_core::Matrix12d A_;       // (Linearized) system matrix
   auv_core::Matrix18d A_Aug_;   // (Linearized) augmented system matrix
   auv_core::Matrix12x8d B_;     // Control input matrix
   auv_core::Matrix18x8d B_Aug_; // Augmented control input matrix
   auv_core::Matrix8x12d K_;     // Gain matrix
   auv_core::Matrix8x18d K_Aug_; // Augmented gain matrix
   auv_core::Matrix12d Q_;       // State cost matrix
   auv_core::Matrix18d Q_Aug_;   // Augmented state cost matrix
   auv_core::Matrix8d R_;        // Input cost matrix

   // LQR Variables
   auv_core::Vector8d totalThrust_;
   auv_core::Vector8d lqrThrust_;
   auv_core::Vector12d error_;
   auv_core::Vector18d augError_;
   Eigen::Vector3d posIntegratorError_;
   Eigen::Quaterniond qState_, qRef_, qError_, qIntegratorError_;
   bool initLQR_, enableIntegrator_;

   // Ceres Problem
   ceres::Problem problemNominalThrust;
   ceres::Solver::Options optionsNominalThrust;
   ceres::Solver::Summary summaryNominalThrust;
   double nominalThrust_[8];
   double quaternion_[4], uvw_[3], pqr_[3], inertialTransAccel_[3], pqrDot_[3];

   // LQR Solver
   // The ct_optcon LQR solver requires these sizes be defined at compile time
   static const size_t state_dim = 12;
   static const size_t state_dim_aug = 18;
   static const size_t control_dim = 8;
   ct::optcon::LQR<state_dim, control_dim> lqrSolver_;
   ct::optcon::LQR<state_dim_aug, control_dim> lqrAugSolver_;
   
   void computeThrustCoeffs();
   void computeLinearizedSystemMatrix(const Eigen::Ref<const auv_core::Vector13d> &ref);
   void computeLinearizedInputMatrix();

public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   AUVLQR(auv_core::auvParameters *auvParams, double dt);

   void setCostMatrices(const Eigen::Ref<const auv_core::Matrix18d> &Q_Aug,
                        const Eigen::Ref<const auv_core::Matrix8d> &R);
   void setCostMatrixQ(const Eigen::Ref<const auv_core::Matrix18d> &Q_Aug);
   void setCostMatrixR(const Eigen::Ref<const auv_core::Matrix8d> &R);

   void setIntegrator(bool enable);

   auv_core::Vector6d getTotalThrustLoad(const Eigen::Ref<const auv_core::Vector8d> &thrusts);
   auv_core::Vector8d computeThrust(const Eigen::Ref<const auv_core::Vector13d> &state,
                          const Eigen::Ref<const auv_core::Vector13d> &ref,
                          const Eigen::Ref<const auv_core::Vector6d> &accel);
};
} // namespace auv_control

#endif