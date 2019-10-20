#include "auv_control/auv_lqr.hpp"

namespace auv_control
{
AUVLQR::AUVLQR(auv_core::auvParameters *auvParams, double dt)
{
   // Vehicle Properties
   dt_ = dt;
   auvParams_ = auvParams;
   maxThrusters_ = 8;
   numThrusters_ = std::min(auvParams_->numThrusters, maxThrusters_);

   // LQR variables
   A_.setZero();
   A_Aug_.setZero();
   B_.setZero();
   K_.setZero();
   K_Aug_.setZero();
   B_Aug_.setZero();
   Q_.setZero();
   Q_Aug_.setZero();
   R_.setZero();

   // Initialize static part of Q_Aug_
   Q_Aug_.block<3, 3>(auv_core::constants::RSTATE_XI_INT, auv_core::constants::RSTATE_XI) = -Eigen::Matrix3d::Identity();
   Q_Aug_.block<3, 3>(auv_core::constants::RSTATE_Q1_INT, auv_core::constants::RSTATE_Q1) = -Eigen::Matrix3d::Identity();

   totalThrust_.setZero();
   lqrThrust_.setZero();
   error_.setZero();
   augError_.setZero();
   qState_.setIdentity();
   qRef_.setIdentity();
   qError_.setIdentity();
   qIntegratorError_.setIdentity();
   posIntegratorError_.setZero();

   initLQR_ = false;
   enableIntegrator_ = false;

   // Initialize arrays
   for (int i = 0; i < maxThrusters_; i++)
      nominalThrust_[i] = 0;

   for (int i = 0; i < 3; i++)
   {
      quaternion_[i] = 0;
      uvw_[i] = 0;
      pqr_[i] = 0;
      inertialTransAccel_[i] = 0;
      pqrDot_[i] = 0;
   }
   quaternion_[3] = 0;

   AUVLQR::computeThrustCoeffs();
   AUVLQR::computeLinearizedInputMatrix();

   // Initialize ceres problem
   problemNominalThrust.AddResidualBlock(
       new ceres::AutoDiffCostFunction<NominalThrustSolver, 6, 8>(new NominalThrustSolver(
           auvParams_, thrustCoeffs_, quaternion_, uvw_, pqr_, inertialTransAccel_, pqrDot_)),
       NULL, nominalThrust_);
   optionsNominalThrust.max_num_iterations = 100;
   optionsNominalThrust.linear_solver_type = ceres::DENSE_QR;
}

/**
 * \brief Set the thruster coefficients. Each column corresponds to a single thruster.
 * 
 * Rows 1,2,3: Force contribution in the body-frame X, Y, and Z axes, respectively (as a fraction)
 * Rows 4,5,6: Effective moment arms about body-frame the X, Y, and Z axes, respectively
 */
void AUVLQR::computeThrustCoeffs()
{
   thrustCoeffs_.setZero();

   for (int i = 0; i < numThrusters_; i++)
   {
      float psi = auvParams_->thrusterData(3, i) * M_PI / 180;
      float theta = auvParams_->thrusterData(4, i) * M_PI / 180;
      thrustCoeffs_(0, i) = cos(psi) * cos(theta); // cos(psi)cos(theta)
      thrustCoeffs_(1, i) = sin(psi) * cos(theta); // sin(psi)cos(theta)
      thrustCoeffs_(2, i) = -sin(theta);           // -sin(theta)

      // Cross-product
      thrustCoeffs_.block<3, 1>(3, i) = auvParams_->thrusterData.block<3, 1>(0, i).cross(thrustCoeffs_.block<3, 1>(0, i));
   }
}

/**
 * @param Q LQR state cost matrix
 * @param R LQR input cost matrix
 */
void AUVLQR::setCostMatrices(const Eigen::Ref<const auv_core::Matrix12d> &Q,
                             const Eigen::Ref<const auv_core::Matrix18d> &Q_Aug,
                             const Eigen::Ref<const auv_core::Matrix8d> &R)
{
   Q_ = Q;
   Q_Aug_ = Q_Aug;
   R_ = R;
   initLQR_ = true;
}

/**
 * @param enable Enable/disable LQR integral action
 */
void AUVLQR::setIntegrator(bool enable)
{
   enableIntegrator_ = enable;

   if (!enableIntegrator_)
   {
      posIntegratorError_.setZero();
      qIntegratorError_.setIdentity();
   }
}

/**
 * @param state Reference state for a given time instance
 * \brief Compute the Jacobian of the 12x12 system matrix
 */
void AUVLQR::computeLinearizedSystemMatrix(const Eigen::Ref<const auv_core::Vector13d> &ref)
{
   // Variables for Auto Diff.
   size_t n = 12, m = 3;
   ADVectorXd X(n), Xdot(n);
   std::vector<double> jac(n * n); // Create nxn elements to represent a Jacobian matrix ()

   // MUST set X to contain INDEPENDENT variables
   CppAD::Independent(X); // Begin recording sequence

   // Using Eigen::Quaternion: quaternion * vector = rotated vector by the described axis-angle/quaternion
   // --> B-frame vector = quaternion.conjugate() * I-frame vector
   // --> I-frame vector = quaternion * B-frame vector
   Eigen::Quaternion<CppAD::AD<double>> ADquat(ref(auv_core::constants::STATE_Q0), X[auv_core::constants::RSTATE_Q1], X[auv_core::constants::RSTATE_Q2], X[auv_core::constants::RSTATE_Q3]);

   // Translational States
   // 1. Time derivatives of: xI, yI, zI (expressed in I-frame)
   Xdot.segment<3>(auv_core::constants::RSTATE_XI) = ADquat * X.segment<3>(auv_core::constants::RSTATE_U);

   // 2. Time-derivatives of: U, V, W (expressed in B-frame)
   Eigen::Vector3d weight = Eigen::Vector3d::Zero();
   weight(2) = auvParams_->mass * auv_core::constants::GRAVITY - auvParams_->Fb;
   ADVector3d transDrag; // Translation drag accel
   transDrag(0) = auvParams_->dragCoeffs(0, 0) * X[auv_core::constants::RSTATE_U] + auv_core::math_lib::sign(ref(auv_core::constants::STATE_U)) * auvParams_->dragCoeffs(3, 0) * X[auv_core::constants::RSTATE_U] * X[auv_core::constants::RSTATE_U];
   transDrag(1) = auvParams_->dragCoeffs(1, 0) * X[auv_core::constants::RSTATE_V] + auv_core::math_lib::sign(ref(auv_core::constants::STATE_V)) * auvParams_->dragCoeffs(4, 0) * X[auv_core::constants::RSTATE_V] * X[auv_core::constants::RSTATE_V];
   transDrag(2) = auvParams_->dragCoeffs(2, 0) * X[auv_core::constants::RSTATE_W] + auv_core::math_lib::sign(ref(auv_core::constants::STATE_W)) * auvParams_->dragCoeffs(5, 0) * X[auv_core::constants::RSTATE_W] * X[auv_core::constants::RSTATE_W];
   Xdot.segment<3>(auv_core::constants::RSTATE_U) = ((ADquat.conjugate() * weight) - transDrag - (X.segment<3>(auv_core::constants::RSTATE_P).cross(X.segment<3>(auv_core::constants::RSTATE_U)))) / auvParams_->mass;

   // Rotational States
   // 3. Time Derivatives of: q1, q2, q3 (remember, quaternion represents the B-frame orientation wrt to the I-frame)
   ADMatrix3d qoIdentity;
   Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
   qoIdentity = identity * ref(auv_core::constants::STATE_Q0);
   Xdot.segment<3>(auv_core::constants::RSTATE_Q1) = 0.5 * (qoIdentity * X.segment<3>(auv_core::constants::RSTATE_P) + X.segment<3>(auv_core::constants::RSTATE_Q1).cross(X.segment<3>(auv_core::constants::RSTATE_P)));

   // 4. Time Derivatives of: P, Q, R (expressed in B-frame)
   Eigen::Vector3d forceBuoyancy = Eigen::Vector3d::Zero();
   forceBuoyancy(2) = -auvParams_->Fb;
   ADVector3d rotDrag; // Rotational drag accel
   rotDrag(0) = auvParams_->dragCoeffs(0, 1) * X[auv_core::constants::RSTATE_P] + auv_core::math_lib::sign(ref(auv_core::constants::STATE_P)) * auvParams_->dragCoeffs(3, 1) * X[auv_core::constants::RSTATE_P] * X[auv_core::constants::RSTATE_P];
   rotDrag(1) = auvParams_->dragCoeffs(1, 1) * X[auv_core::constants::RSTATE_Q] + auv_core::math_lib::sign(ref(auv_core::constants::STATE_Q)) * auvParams_->dragCoeffs(4, 1) * X[auv_core::constants::RSTATE_Q] * X[auv_core::constants::RSTATE_Q];
   rotDrag(2) = auvParams_->dragCoeffs(2, 1) * X[auv_core::constants::RSTATE_R] + auv_core::math_lib::sign(ref(auv_core::constants::STATE_R)) * auvParams_->dragCoeffs(5, 1) * X[auv_core::constants::RSTATE_R] * X[auv_core::constants::RSTATE_R];
   Xdot.segment<3>(auv_core::constants::RSTATE_P) = auvParams_->inertia.inverse() * (auvParams_->cob.cross(ADquat.conjugate() * forceBuoyancy) - rotDrag - X.segment<3>(auv_core::constants::RSTATE_P).cross(auvParams_->inertia * X.segment<3>(auv_core::constants::RSTATE_P)));

   // Reduced state (excludes q0)
   auv_core::Vector12d reducedState;
   reducedState.head<6>() = reducedState.head<6>();
   reducedState.tail<6>() = reducedState.tail<6>();

   // Compute Jacobian
   std::vector<double> x;
   for (int i = 0; i < 12; i++)
      x.push_back(reducedState(i));
   CppAD::ADFun<double> f(X, Xdot);
   jac = f.Jacobian(x);

   // Put jacobien elements into matrix format from vector
   A_.setZero();
   for (int i = 0; i < 12; i++)
      for (int j = 0; j < 12; j++)
         A_(i, j) = (double)jac[i * 12 + j];

   if (enableIntegrator_)
      A_Aug_.block<12, 12>(0, 0) = A_;
}

/**
 * \brief Compute the control input matrix.
 */
void AUVLQR::computeLinearizedInputMatrix()
{
   B_.block<3, 8>(auv_core::constants::RSTATE_U, 0) = thrustCoeffs_.block<3, 8>(0, 0);                                     // Force contributions
   B_.block<3, 8>(auv_core::constants::RSTATE_P, 0) = auvParams_->inertia.inverse() * thrustCoeffs_.block<3, 8>(3, 0);     // Moment contributions
   B_Aug_.block<3, 8>(auv_core::constants::RSTATE_U, 0) = thrustCoeffs_.block<3, 8>(0, 0);                                 // Force contributions
   B_Aug_.block<3, 8>(auv_core::constants::RSTATE_P, 0) = auvParams_->inertia.inverse() * thrustCoeffs_.block<3, 8>(3, 0); // Moment contributions
}

// Get total thruster forces/moments as expressed in the B-frame
// Parameters:
//      thrusts = VectorXf of force exerted on vehicle by each thruster
auv_core::Vector6d AUVLQR::getTotalThrustLoad(const Eigen::Ref<const auv_core::Vector8d> &thrusts)
{
   auv_core::Vector6d thrustLoad;
   thrustLoad.setZero();

   for (int i = 0; i < numThrusters_; i++)
      thrustLoad = thrustLoad + thrusts(i) * thrustCoeffs_.col(i);

   return thrustLoad;
}

auv_core::Vector8d AUVLQR::computeThrust(const Eigen::Ref<const auv_core::Vector13d> &state,
                                         const Eigen::Ref<const auv_core::Vector13d> &ref,
                                         const Eigen::Ref<const auv_core::Vector6d> &accel)
{
   lqrThrust_.setZero();
   totalThrust_.setZero();

   // Set variables for nominal thrust solver
   quaternion_[0] = ref(auv_core::constants::STATE_Q0);
   quaternion_[1] = ref(auv_core::constants::STATE_Q1);
   quaternion_[2] = ref(auv_core::constants::STATE_Q2);
   quaternion_[3] = ref(auv_core::constants::STATE_Q3);

   Eigen::Map<Eigen::Vector3d>(&uvw_[0], 3, 1) = ref.segment<3>(auv_core::constants::STATE_U);
   Eigen::Map<Eigen::Vector3d>(&pqr_[0], 3, 1) = ref.segment<3>(auv_core::constants::STATE_P);
   Eigen::Map<Eigen::Vector3d>(&inertialTransAccel_[0], 3, 1) = accel.head<3>();
   Eigen::Map<Eigen::Vector3d>(&pqrDot_[0], 3, 1) = accel.tail<3>();

   // Initialize nominal forces to zero (this doesn't make too much of a difference)
   for (int i = 0; i < maxThrusters_; i++)
      nominalThrust_[i] = 0;

   if (initLQR_)
   {
      ceres::Solve(optionsNominalThrust, &problemNominalThrust, &summaryNominalThrust);
      Eigen::Map<auv_core::Vector8d> nominalThrust(nominalThrust_);
      AUVLQR::computeLinearizedSystemMatrix(ref);

      qState_ = Eigen::Quaterniond(state(auv_core::constants::STATE_Q0), state(auv_core::constants::STATE_Q1), state(auv_core::constants::STATE_Q2), state(auv_core::constants::STATE_Q3));
      qRef_ = Eigen::Quaterniond(ref(auv_core::constants::STATE_Q0), ref(auv_core::constants::STATE_Q1), ref(auv_core::constants::STATE_Q2), ref(auv_core::constants::STATE_Q3));
      qError_ = qRef_ * qState_.conjugate(); //qState * qRef.conjugate(); // Want quaternion error relative to Inertial-frame (is it qRef * qState.conjugate()?)

      error_.head<6>() = state.head<6>() - ref.head<6>();
      error_.tail<6>() = state.tail<6>() - ref.tail<6>();
      error_.segment<3>(auv_core::constants::RSTATE_Q1) = -qError_.vec(); // Set quaternion error, negate vector part for calculation of input vector

      if (!enableIntegrator_)
      {
         //std::cout << "LQR error: " << error << std::endl;
         lqrSolver_.compute(Q_, R_, A_, B_, K_);
         lqrThrust_ = -K_ * error_; // U = -K*(state-ref)
      }
      else
      { // TODO: Verify quaternion error calculation, or improve if not correct
         augError_.head<12>() = error_;
         qIntegratorError_ = qIntegratorError_ * qError_; // Regard integral error of quaternion as another quaternion
         posIntegratorError_ = posIntegratorError_ + dt_ * augError_.segment<3>(auv_core::constants::RSTATE_XI);
         augError_.segment<3>(auv_core::constants::RSTATE_XI_INT) = posIntegratorError_;
         augError_.segment<3>(auv_core::constants::RSTATE_Q1_INT) = -qIntegratorError_.vec();

         lqrAugSolver_.compute(Q_Aug_, R_, A_Aug_, B_Aug_, K_Aug_);
         lqrThrust_ = -K_Aug_ * augError_; // U = -K*(state-ref)
                                           // Add integral error !!!!!!!!!!!
      }

      //std::cout << "Solve LQR, A matrix: " << std::endl << A_ << std::endl;
      //std::cout << "Solve LQR, B matrix: " << std::endl << B_ << std::endl;
      //std::cout << "Reference state: " << std::endl << ref << std::endl;
      //std::cout << "Accel state: " << std::endl << accel << std::endl;
      //std::cout << "LQR gain K: " << K_ << std::endl;
      //std::cout << "Nominal Thrust: " << std::endl << nominalThrust << std::endl;
      std::cout << "LQR thrust: " << lqrThrust_ << std::endl;

      totalThrust_ = nominalThrust + lqrThrust_;
   }
   return totalThrust_;
}
} // namespace auv_control
