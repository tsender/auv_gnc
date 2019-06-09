#include "auv_control/auv_model.hpp"

namespace AUVControl
{
AUVModel::AUVModel(double mass, double volume, double fluid_density, const Ref<const Matrix3d> &inertia, const Ref<const Vector3d> &CoB,
                   const Ref<const Matrix62d> &dragCoeffs, const Ref<const Matrix5Xd> &thrusters)
{
    mass_ = mass;             // [kg]
    inertia_ = inertia;       // 3x3 inertia matrix
    volume_ = volume;         // [m^3]
    density_ = fluid_density; // [kg/m^3]
    CoB_ = CoB;
    dragCoeffs_ = dragCoeffs;
    thrusters_ = thrusters;
    numThrusters_ = thrusters_.cols();
    Fg_ = mass_ * GRAVITY;              // Force due to gravity [N]
    Fb_ = density_ * volume_ * GRAVITY; // Buoyant Force [N]

    AUVModel::setThrustCoeffs();
    AUVModel::setLinearizedInputMatrix();

    for (int i = 0; i < 10; i++)
        nominalForces[i] = 0;

    nominalThrustProblem.AddResidualBlock(new ceres::AutoDiffCostFunction<NominalThrustSolver, 6, 10>(new NominalThrustSolver(mass_, volume_, density_, 
                            inertia_, CoB_, dragCoeffs_, thrustCoeffs_, quaternion_, uvw_, pqr_, inertialTransAccel_, pqrDot_)), NULL, nominalForces);
    nominalThrustOptions.max_num_iterations = 100;
    nominalThrustOptions.linear_solver_type = ceres::DENSE_QR;
}

// Set the thruster coefficients. Each column corresponds to a single thruster.
// Rows 1,2,3: force contribution in the X, Y, and Z axes, respectively (range from 0-1)
// Rows 4,5,6: effective moment arms about the X, Y, and Z axes, respectively
void AUVModel::setThrustCoeffs()
{
    thrustCoeffs_.resize(6, numThrusters_);
    thrustCoeffs_.setZero();

    for (int i = 0; i < numThrusters_; i++)
    {
        float psi = thrusters_(3, i) * PI / 180;
        float theta = thrusters_(4, i) * PI / 180;
        thrustCoeffs_(0, i) = cos(psi) * cos(theta); // cos(psi)cos(theta)
        thrustCoeffs_(1, i) = sin(psi) * cos(theta); // sin(psi)cos(theta)
        thrustCoeffs_(2, i) = -sin(theta);           // -sin(theta)

        // Cross-product
        thrustCoeffs_.block<3, 1>(3, i) = thrusters_.block<3, 1>(0, i).cross(thrustCoeffs_.block<3, 1>(0, i));
    }
}

// Get total thruster forces/moments as expressed in the B-frame
// Parameters:
//      thrusts = VectorXf of force exerted on vehicle by each thruster
Vector6d AUVModel::getTotalThrustLoad(const Ref<const VectorXd> &thrusts)
{
    Vector6d thrustLoad;
    thrustLoad.setZero();

    if (thrusts.rows() == numThrusters_)
        for (int i = 0; i < numThrusters_; i++)
            thrustLoad = thrustLoad + thrusts(i) * thrustCoeffs_.col(i);

    return thrustLoad;
}

/*// Get forces/moments due to to vehicle's weight and buoyancy as expressed in the B-frame
// Parameters:
//      attitude = Eigen::Vector3f of roll, pitch, and yaw [rad] (in this order)
Vector6d AUVModel::getWeightLoad(const Ref<const Vector3d> &attitude)
{
    Vector6f weightLoad;
    weightLoad.setZero();

    Vector3f coeffs; // Store coefficients here
    coeffs.setZero();
    float theta = attitude(1);
    float phi = attitude(2);

    coeffs(0) = -sin(theta);
    coeffs(1) = sin(phi) * cos(theta);
    coeffs(2) = cos(phi) * cos(theta);

    weightLoad.head<3>() = (Fg_ - Fb_) * coeffs;        // Forces, expressed in  B-frame
    weightLoad.tail<3>() = CoB_.cross((-Fb_ * coeffs)); // Moments, expressed in B-frame

    return weightLoad;
}*/

/**
 * \param ref Reference state for a given time instance
 * \brief Compute the 12x12 Jacobian of the A-matrix
 */
Matrix12d AUVModel::getLinearizedSystemMatrix(const Ref<const Vector12d> &ref)
{
    double q0 = sqrt(1.0 - pow(ref(q1_), 2) + pow(ref(q2_), 2) + pow(ref(q3_), 2)); // Get q0 from unit quaternion

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
    Eigen::Quaternion<CppAD::AD<double> > ADquat(q0, X[q1_], X[q2_], X[q3_]);

    // Translational States
    // 1. Time derivatives of: xI, yI, zI (expressed in I-frame)
    Xdot.head<3>() = ADquat * Xdot.segment<3>(U_);

    // 2. Time-derivatives of: U, V, W (expressed in B-frame)
    Vector3d weightAccel = Vector3d::Zero();
    weightAccel(2) = (Fg_ - Fb_) / mass_;
    ADVector3d transDrag; // Translation drag accel
    transDrag(0) = (dragCoeffs_(0, 0) * X[U_] + 0.5 * AUVMathLib::sign(ref(U_)) * density_ * dragCoeffs_(0, 1) * X[U_] * X[U_]) / mass_;
    transDrag(1) = (dragCoeffs_(1, 0) * X[V_] + 0.5 * AUVMathLib::sign(ref(V_)) * density_ * dragCoeffs_(1, 1) * X[V_] * X[V_]) / mass_;
    transDrag(2) = (dragCoeffs_(2, 0) * X[W_] + 0.5 * AUVMathLib::sign(ref(W_)) * density_ * dragCoeffs_(2, 1) * X[W_] * X[W_]) / mass_;
    Xdot.segment<3>(U_) = (ADquat.conjugate() * weightAccel) - transDrag - (X.segment<3>(P_).cross(X.segment<3>(U_)));

    // Rotational States
    // 3. Time Derivatives of: q1, q2, q3 (remember, quaternion represents the B-frame orientation wrt to the I-frame)
    ADMatrix3d qoIdentity;
    Matrix3d identity = Matrix3d::Identity();
    qoIdentity = identity * q0;
    Xdot.segment<3>(q1_) = 0.5 * (qoIdentity * X.segment<3>(P_) + X.segment<3>(q1_).cross(X.segment<3>(P_)));

    // 4. Time Derivatives of: P, Q, R (expressed in B-frame)
    Vector3d forceBuoyancy = Vector3d::Zero();
    forceBuoyancy(2) = -Fb_;
    ADVector3d rotDrag; // Rotational drag accel
    rotDrag(0) = (dragCoeffs_(3, 0) * X[P_] + 0.5 * AUVMathLib::sign(ref(P_)) * density_ * dragCoeffs_(3, 1) * X[P_] * X[P_]);
    rotDrag(1) = (dragCoeffs_(4, 0) * X[Q_] + 0.5 * AUVMathLib::sign(ref(Q_)) * density_ * dragCoeffs_(4, 1) * X[Q_] * X[Q_]);
    rotDrag(2) = (dragCoeffs_(5, 0) * X[R_] + 0.5 * AUVMathLib::sign(ref(R_)) * density_ * dragCoeffs_(5, 1) * X[R_] * X[R_]);
    Xdot.segment<3>(P_) = inertia_.inverse() * (-rotDrag + CoB_.cross(ADquat.conjugate() * forceBuoyancy) - X.segment<3>(P_).cross(inertia_ * X.segment<3>(P_)));

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
    Xdot[U_] = (-dragCoeffs_(0, 0) * X[U_] - 0.5 * AUVMathLib::sign(ref(U_)) * density_ * dragCoeffs_(0, 1) * X[U_] * X[U_]) / mass_ -
               ((Fg_ - Fb_) / mass_) * CppAD::sin(X[theta_]) - transportUVW(1);

    Xdot[V_] = (-dragCoeffs_(1, 0) * X[V_] - 0.5 * AUVMathLib::sign(ref(V_)) * density_ * dragCoeffs_(1, 1) * X[V_] * X[V_]) / mass_ +
               ((Fg_ - Fb_) / mass_) * CppAD::cos(X[theta_]) * CppAD::sin(X[phi_]) - transportUVW(2);

    Xdot[W_] = (-dragCoeffs_(2, 0) * X[W_] - 0.5 * AUVMathLib::sign(ref(W_)) * density_ * dragCoeffs_(2, 1) * X[W_] * X[W_]) / mass_ +
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

    Matrix12d A;
    A.setZero();

    // Put jac elements into matrix format
    for (int i = 0; i < 12; i++)
        for (int j = 0; j < 12; j++)
            A(i, j) = (float)jac[i * 12 + j];

    return A;
}

/**
 * \brief Set the control input matrix.
 */
void AUVModel::setLinearizedInputMatrix()
{
    B_.resize(12, numThrusters_);
    B_.setZero();
    B_.block(U_, 0, 3, numThrusters_) = thrustCoeffs_.block(0, 0, 3, numThrusters_);
    B_.block(P_, 0, 3, numThrusters_) = inertia_.inverse() * thrustCoeffs_.block(3, 0, 3, numThrusters_);
}

/**
 * \brief Return the current control input matrix.
 */
Matrix12Xd AUVModel::getLinearizedInputMatrix()
{
    return B_;
}
} // namespace AUVControl
