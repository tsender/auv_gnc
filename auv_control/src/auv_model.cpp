#include "auv_control/auv_model.hpp"

namespace auv_control
{
AUVModel::AUVModel(double Fg, double Fb,
                   const Eigen::Ref<const Eigen::Vector3d> &CoB,
                   const Eigen::Ref<const Eigen::Matrix3d> &inertia,
                   const Eigen::Ref<const Matrix62d> &dragCoeffs,
                   const Eigen::Ref<const Matrix58d> &thrusters,
                   int numThrusters)
{
    // Vehicle Properties
    Fg_ = Fg;                        // [N]
    Fb = Fb;                         // [N]
    mass_ = Fg_ / AUVModel::GRAVITY; // [kg]
    CoB_ = CoB;                      // Center of buoyancy relative to center of mass (X [m], Y [m], Z [m])
    inertia_ = inertia;              // 3x3 inertia matrix [kg-m^2]
    dragCoeffs_ = dragCoeffs;
    thrusters_ = thrusters;
    maxThrusters_ = 8;
    numThrusters_ = std::min(numThrusters, maxThrusters_);

    // LQR variables
    A_.setZero();
    B_.setZero();
    Q_.setZero();
    R_.setZero();
    initLQR_ = false;

    // Initialize arrays
    for (int i = 0; i < maxThrusters_; i++)
        nominalForces[i] = 0;

    for (int i = 0; i < 3; i++)
    {
        quaternion_[i] = 0;
        uvw_[i] = 0;
        pqr_[i] = 0;
        inertialTransAccel_[i] = 0;
        pqrDot_[0] = 0;
    }
    quaternion_[3] = 0;

    // Initialize ceres problem
    problemNominalThrust.AddResidualBlock(
        new ceres::AutoDiffCostFunction<NominalThrustSolver, 6, 8>(new NominalThrustSolver(Fg_, Fb_, CoB_, inertia_, dragCoeffs_, thrustCoeffs_,
                                                                                           quaternion_, uvw_, pqr_, inertialTransAccel_, pqrDot_)),
        NULL, nominalForces);
    optionsNominalThrust.max_num_iterations = 100;
    optionsNominalThrust.linear_solver_type = ceres::DENSE_QR;

    AUVModel::setThrustCoeffs();
    AUVModel::setLinearizedInputMatrix();
}

// Set the thruster coefficients. Each column corresponds to a single thruster.
// Rows 1,2,3: force contribution in the X, Y, and Z axes, respectively (range from 0-1)
// Rows 4,5,6: effective moment arms about the X, Y, and Z axes, respectively
void AUVModel::setThrustCoeffs()
{
    //thrustCoeffs_.resize(6, numThrusters_);
    thrustCoeffs_.setZero();

    for (int i = 0; i < numThrusters_; i++)
    {
        float psi = thrusters_(3, i) * M_PI / 180;
        float theta = thrusters_(4, i) * M_PI / 180;
        thrustCoeffs_(0, i) = cos(psi) * cos(theta); // cos(psi)cos(theta)
        thrustCoeffs_(1, i) = sin(psi) * cos(theta); // sin(psi)cos(theta)
        thrustCoeffs_(2, i) = -sin(theta);           // -sin(theta)

        // Cross-product
        thrustCoeffs_.block<3, 1>(3, i) = thrusters_.block<3, 1>(0, i).cross(thrustCoeffs_.block<3, 1>(0, i));
    }
}

/**
 * @param Q LQR state cost matrix
 * @param R LQR input cost matrix
 */
void AUVModel::setLQRCostMatrices(const Eigen::Ref<const Matrix12d> &Q, const Eigen::Ref<const Matrix8d> &R)
{
    Q_ = Q;
    R_ = R;
    initLQR_ = true;
}

/**
 * \param state Reference state for a given time instance
 * \brief Compute the 12x12 Jacobian of the A-matrix
 */
void AUVModel::setLinearizedSystemMatrix(const Eigen::Ref<const Vector12d> &ref)
{
    // Get unit quaternions
    //double q0State = sqrt(1.0 - pow(state(STATE_Q1), 2) + pow(state(STATE_Q2), 2) + pow(state(STATE_Q3), 2));
    double q0Ref = sqrt(1.0 - pow(ref(STATE_Q1), 2) + pow(ref(STATE_Q2), 2) + pow(ref(STATE_Q3), 2));

    // Eigen::Quaterniond qState(q0State, state(STATE_Q1), state(STATE_Q2), state(STATE_Q2));
    // Eigen::Quaterniond qRef(q0Ref, ref(STATE_Q1), ref(STATE_Q2), ref(STATE_Q2));
    // Eigen::Quaterniond qError = qState.conjugate() * qRef;

    // Evaluation vector - replace quaternion with error quaternion values
    // Vector12d eval = ref;
    // eval.block<3>(STATE_Q1) = qError.vec();

    // Variables for Auto Diff.
    size_t n = 12, m = 3;
    ADVectorXd X(n), Xdot(n);
    std::vector<double> jac(n * n); // Create nxn Jacobian matrix

    // MUST set X to contain INDEPENDENT variables
    CppAD::Independent(X); // Begin recording sequence

    /*ADMatrix3d ADRquat; // This math converts I-frame vector to B-frame vector
    ADRquat(0, 0) = q0 * q0 + X[q1_] * X[q1_] - X[q2_] * X[q2_] - X[q3_] * X[q3_];
    ADRquat(1, 1) = q0 * q0 - X[q1_] * X[q1_] + X[q2_] * X[q2_] - X[q3_] * X[q3_];
    ADRquat(2, 2) = q0 * q0 - X[q1_] * X[q1_] - X[q2_] * X[q2_] + X[q3_] * X[q3_];
    ADRquat(0, 1) = 2 * X[q1_] * X[q2_] + 2 * q0 * X[q3_];
    ADRquat(1, 0) = 2 * X[q1_] * X[q2_] - 2 * q0 * X[q3_];
    ADRquat(0, 2) = 2 * X[q1_] * X[q3_] - 2 * q0 * X[q2_];
    ADRquat(2, 0) = 2 * X[q1_] * X[q3_] + 2 * q0 * X[q2_];
    ADRquat(1, 2) = 2 * X[q2_] * X[q3_] + 2 * q0 * X[q1_];
    ADRquat(2, 1) = 2 * X[q2_] * X[q3_] - 2 * q0 * X[q1_];*/

    // Using Eigen::Quaternion: quaternion * vector = rotates vector by the axis-angle specified
    // So: B-frame vector = quaternion.conjugate() * I-frame vector
    // So: I-frame vector = quaternion * B-frame vector
    Eigen::Quaternion<CppAD::AD<double>> ADquat(q0Ref, X[STATE_Q1], X[STATE_Q2], X[STATE_Q3]);

    // Translational States
    // 1. Time derivatives of: xI, yI, zI (expressed in I-frame)
    Xdot.head<3>() = ADquat * Xdot.segment<3>(STATE_U);

    // 2. Time-derivatives of: U, V, W (expressed in B-frame)
    Eigen::Vector3d weight = Eigen::Vector3d::Zero();
    weight(2) = Fg_ - Fb_;
    ADVector3d transDrag; // Translation drag accel
    transDrag(0) = dragCoeffs_(0, 0) * X[STATE_U] + auv_math_lib::sign(ref(STATE_U)) * dragCoeffs_(3, 0) * X[STATE_U] * X[STATE_U];
    transDrag(1) = dragCoeffs_(1, 0) * X[STATE_V] + auv_math_lib::sign(ref(STATE_V)) * dragCoeffs_(4, 0) * X[STATE_V] * X[STATE_V];
    transDrag(2) = dragCoeffs_(2, 0) * X[STATE_W] + auv_math_lib::sign(ref(STATE_W)) * dragCoeffs_(5, 0) * X[STATE_W] * X[STATE_W];
    Xdot.segment<3>(STATE_U) = ((ADquat.conjugate() * weight) - transDrag - (X.segment<3>(STATE_P).cross(X.segment<3>(STATE_U)))) / mass_;

    // Rotational States
    // 3. Time Derivatives of: q1, q2, q3 (remember, quaternion represents the B-frame orientation wrt to the I-frame)
    ADMatrix3d qoIdentity;
    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    qoIdentity = identity * q0Ref;
    Xdot.segment<3>(STATE_Q1) = 0.5 * (qoIdentity * X.segment<3>(STATE_P) + X.segment<3>(STATE_Q1).cross(X.segment<3>(STATE_P)));

    // 4. Time Derivatives of: P, Q, R (expressed in B-frame)
    Eigen::Vector3d forceBuoyancy = Eigen::Vector3d::Zero();
    forceBuoyancy(2) = -Fb_;
    ADVector3d rotDrag; // Rotational drag accel
    rotDrag(0) = dragCoeffs_(0, 1) * X[STATE_P] + auv_math_lib::sign(ref(STATE_P)) * dragCoeffs_(3, 1) * X[STATE_P] * X[STATE_P];
    rotDrag(1) = dragCoeffs_(1, 1) * X[STATE_Q] + auv_math_lib::sign(ref(STATE_Q)) * dragCoeffs_(4, 1) * X[STATE_Q] * X[STATE_Q];
    rotDrag(2) = dragCoeffs_(2, 1) * X[STATE_R] + auv_math_lib::sign(ref(STATE_R)) * dragCoeffs_(5, 1) * X[STATE_R] * X[STATE_R];
    Xdot.segment<3>(STATE_P) = inertia_.inverse() * (CoB_.cross(ADquat.conjugate() * forceBuoyancy) - rotDrag - X.segment<3>(STATE_P).cross(inertia_ * X.segment<3>(STATE_P)));

    // Get Rotation Matrices (Matrices filled by column-order)
    /*ADMatrixXd ADRotX, ADRotY, ADRotZ, ADRoti2b, ADRotb2i;
    ADRotX << 1, 0, 0, 0, CppAD::cos(X[phi_]), -CppAD::sin(X[phi_]), 0, CppAD::sin(X[phi_]), CppAD::cos(X[phi_]);
    ADRotY << CppAD::cos(X[theta_]), 0, CppAD::sin(X[theta_]), 0, 1, 0, -CppAD::sin(X[theta_]), 0, CppAD::cos(X[theta_]);
    ADRotZ << CppAD::cos(X[psi_]), -CppAD::sin(X[psi_]), 0, CppAD::sin(X[psi_]), CppAD::cos(X[psi_]), 0, 0, 0, 1;
    ADRoti2b = ADRotX * ADRotY * ADRotZ; // Inertial to Body
    ADRotb2i = ADRoti2b.transpose();     // Body to Inertial

    // 1. Time derivatives of: xI, yI, zI,
    Xdot.head<3>() = ADRotb2i * X.segment<3>(U_);

    // 2. Time-derivatives of: psi, theta, and phi
    Xdot[phi_] = X[P_] + (X[Q_] * CppAD::sin(X[phi_]) + X[R_] * CppAD::cos(X[phi_])) * CppAD::tan(X[theta_]);

    Xdot[theta_] = X[Q_] * CppAD::cos(X[phi_]) - X[R_] * CppAD::sin(X[phi_]);

    Xdot[psi_] = (X[Q_] * CppAD::sin(X[phi_]) + X[R_] * CppAD::cos(X[phi_])) / cos(X[theta_]);

    // 3. Time-derivatives of : U, V, W
    ADVectorXd transportUVW = X.segment<3>(P_).cross(X.segment<3>(U_)); // [p, q, r] x [u, v, w]
    Xdot[U_] = (-dragCoeffs_(0, 0) * X[U_] - 0.5 * auv_math_lib::sign(ref(U_)) * density_ * dragCoeffs_(0, 1) * X[U_] * X[U_]) / mass_ -
               ((Fg_ - Fb_) / mass_) * CppAD::sin(X[theta_]) - transportUVW(1);

    Xdot[V_] = (-dragCoeffs_(1, 0) * X[V_] - 0.5 * auv_math_lib::sign(ref(V_)) * density_ * dragCoeffs_(1, 1) * X[V_] * X[V_]) / mass_ +
               ((Fg_ - Fb_) / mass_) * CppAD::cos(X[theta_]) * CppAD::sin(X[phi_]) - transportUVW(2);

    Xdot[W_] = (-dragCoeffs_(2, 0) * X[W_] - 0.5 * auv_math_lib::sign(ref(W_)) * density_ * dragCoeffs_(2, 1) * X[W_] * X[W_]) / mass_ +
               ((Fg_ - Fb_) / mass_) * CppAD::cos(X[theta_]) * CppAD::cos(X[phi_]) - transportUVW(3);

    // 4. Time-derivatives of: P, Q, R
    ADVectorXd transportPQR;
    transportPQR = X.segment<3>(P_).cross(inertia_ * X.segment<3>(P_)); // [p, q, r] x I*[p, q, r]
    Xdot.segment<3>(P_) = -inertia_.inverse() * transportPQR;*/

    // Compute Jacobian
    std::vector<double> x;
    for (int i = 0; i < 12; i++)
        x.push_back(ref(i));
    CppAD::ADFun<double> f(X, Xdot);
    jac = f.Jacobian(x);

    A_.setZero();
    // Put jac elements into matrix format
    for (int i = 0; i < 12; i++)
        for (int j = 0; j < 12; j++)
            A_(i, j) = (float)jac[i * 12 + j];
}

/**
 * \brief Set the control input matrix.
 */
void AUVModel::setLinearizedInputMatrix()
{
    B_.setZero();
    B_.block<3, 8>(STATE_U, 0) = thrustCoeffs_.block<3, 8>(0, 0);
    B_.block<3, 8>(STATE_P, 0) = inertia_.inverse() * thrustCoeffs_.block<3, 8>(3, 0);
}

/**
 * \brief Return the current system matrix.
 */
Matrix12d AUVModel::getLinearizedSystemMatrix()
{
    return A_;
}

/**
 * \brief Return the current control input matrix.
 */
Matrix12x8d AUVModel::getLinearizedInputMatrix()
{
    return B_;
}

// Get total thruster forces/moments as expressed in the B-frame
// Parameters:
//      thrusts = VectorXf of force exerted on vehicle by each thruster
Vector6d AUVModel::getTotalThrustLoad(const Eigen::Ref<const Vector8d> &thrusts)
{
    Vector6d thrustLoad;
    thrustLoad.setZero();

    for (int i = 0; i < numThrusters_; i++)
        thrustLoad = thrustLoad + thrusts(i) * thrustCoeffs_.col(i);

    return thrustLoad;
}

Vector8d AUVModel::computeLQRThrust(const Eigen::Ref<const Vector12d> &state,
                                    const Eigen::Ref<const Vector12d> &ref,
                                    const Eigen::Ref<const Vector6d> &accel)
{
    Vector8d totalThrust;
    totalThrust.setZero();

    // Set variables for nominal thrust solver
    double q0State = sqrt(1.0 - pow(state(STATE_Q1), 2) + pow(state(STATE_Q2), 2) + pow(state(STATE_Q3), 2));
    double q0Ref = sqrt(1.0 - pow(ref(STATE_Q1), 2) + pow(ref(STATE_Q2), 2) + pow(ref(STATE_Q3), 2));
    quaternion_[0] = q0Ref;
    quaternion_[1] = ref(STATE_Q1);
    quaternion_[2] = ref(STATE_Q2);
    quaternion_[3] = ref(STATE_Q3);

    Eigen::Map<Eigen::Vector3d>(&uvw_[0], 3, 1) = ref.segment<3>(STATE_U);
    Eigen::Map<Eigen::Vector3d>(&pqr_[0], 3, 1) = ref.segment<3>(STATE_P);
    Eigen::Map<Eigen::Vector3d>(&inertialTransAccel_[0], 3, 1) = accel.head<3>();
    Eigen::Map<Eigen::Vector3d>(&pqrDot_[0], 3, 1) = accel.tail<3>();

    // Initialize nominal forces to zero (this doesn't make too much of a difference)
    for (int i = 0; i < 10; i++)
        nominalForces[i] = 0;

    if (initLQR_)
    {
        ceres::Solve(optionsNominalThrust, &problemNominalThrust, &summaryNominalThrust);
        Eigen::Map<Vector8d> nominalThrust(nominalForces);

        Eigen::Quaterniond qState(q0State, state(STATE_Q1), state(STATE_Q2), state(STATE_Q3));
        Eigen::Quaterniond qRef(q0Ref, ref(STATE_Q1), ref(STATE_Q2), ref(STATE_Q3));
        Eigen::Quaterniond qError = qState.conjugate() * qRef;
        Vector12d error = state - ref;
        error.segment<3>(STATE_Q1) = qError.vec();

        Vector8d lqrThrust;
        lqrThrust.setZero();
        lqrSolver_.compute(Q_, R_, A_, B_, K_);
        lqrThrust = -K_ * error; // U = -K*(state-ref)

        totalThrust = nominalThrust + lqrThrust;
    }
    return totalThrust;
}
} // namespace auv_control
