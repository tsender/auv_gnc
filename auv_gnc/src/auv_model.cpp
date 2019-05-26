#include "riptide_gnc/auv_model.h"

AUModel::AUVModel(float m, float V, float fluid_rho, const Ref<const Matrix3f> &Inertia, const Ref<const Vector3f> &cob,
                  const Ref<const Matrix62f> &drag, vector<Vector5f> &auv_thrusters);
{
    mass = m;          // [kg]
    inertia = Inertia; // 3x3 inertia matrix
    vol = V;           // [m^3]
    rho = fluid_rho;   // [kg/m^3]
    CoB = cob;
    dragCoeffs = drag;
    thrusters = auv_thrusters;
    numThrusters = thrusters.cols();
    Fg = mass * GRAVITY;      // Force due to gravity [N]
    Fb = rho * vol * GRAVITY; // Buoyant Force [N]

    // Get primary and products of inertia elements
    Ixx = inertia(0, 0);
    Iyy = inertia(1, 1);
    Izz = inertia(2, 2);
    Ixy = -inertia(0, 1);
    Ixz = -inertia(0, 2);
    Iyz = -inertia(1, 2);

    AUVModel::SetThrustCoeffs();
}

// Set the thruster coefficients. Each column corresponds to a single thruster.
// Rows 1,2,3: force contribution in the X, Y, and Z axes, respectively
// Rows 4,5,6: effective moment arms about the X, Y, and Z axes, respectively
void AUVModel::SetThrustCoeffs()
{
    thrustCoeffs.resize(6, numThrusters);
    thrustCoeffs.setZero();

    for (int i = 0; i < numThrusters; i++)
    {
        float psi = thrusters(3, i) * PI / 180;
        float theta = thrusters(4, i) * PI / 180;
        thrustCoeffs(0, i) = cos(psi) * cos(theta); // cos(psi)cos(theta)
        thrustCoeffs(1, i) = sin(psi) * cos(theta); // sin(psi)cos(theta)
        thrustCoeffs(2, i) = -sin(theta);           // -sin(theta)

        // Cross-product
        thrustCoeffs.block<3, 1>(3, i) = thrusters.block<3, 1>(0, i).cross(thrustCoeffs.block<3, 1>(0, i));
    }
}

// Get total thruster forces/moments as expressed in the B-frame
// Parameters:
//      thrusts = VectorXf of force exerted on vehicle by each thruster
Vector6f AUVModel::GetTotalThrustLoad(const Ref<const VectorXf> &thrusts)
{
    Vector6f thrustLoad;
    thrustLoad.setZero();

    if (thrusts.rows() == numThrusters)
        for (int i = 0; i < numThrusters; i++)
            thrustLoad = thrustLoad + thrusts(i) * thrustCoeffs.col(i);

    return thrustLoad;
}

// Get forces/moments due to to vehicle's weight and buoyancy as expressed in the B-frame
// Parameters:
//      attitude = Eigen::Vector3f of yaw, pitch, and roll (in this order)
Vector6f AUVModel::GetWeightLoad(const Ref<const Vector3f> &attitude)
{
    Vector6f weightLoad;
    weightLoads.setZero();

    Vector3f coeffs; // Store coefficients here
    coeffs.setZero();
    float phi = attitude(2);
    float theta = attitude(1);

    coeffs(0) = -sin(theta);
    coeffs(1) = sin(phi) * cos(theta);
    coeffs(2) = cos(phi) * cos(theta);

    weightLoad.head<3>() = (Fg - Fb) * coeffs;        // Forces, expressed in  B-frame
    weightLoad.tail<3>() = CoB.cross((-Fb * coeffs)); // Moments, expressed in B-frame

    return weightLoad;
}

// Compute the 12x12 Jacobian of the A-matrix
// DO NOT CHANGE !!!!!!
Matrix12f AUVModel::GetStateJacobian(const Ref<const Vector12f> &ref)
{
    // Variables for Auto Diff.
    size_t n = 12, m = 3;
    ADVectorXd X(n), Xdot(n);
    vector<double> jac(n * n); // Create nxn Jacobian matrix

    // MUST set X to contain INDEPENDENT variables
    // Calling this function will BEGIN the recording sequence
    CppAD::Independent(X);

    // Get Rotation Matrices (Matrices filled by column-order)
    ADMatrixXd ADRotX << 1, 0, 0, 0, CppAD::cos(X(_phi])), -CppAD::sin(X(_phi])), 0, CppAD::sin(X(_phi])), CppAD::cos(X(_phi]));
    ADMatrixXd ADRotY << CppAD::cos(X(_theta])), 0, CppAD::sin(X(_theta])), 0, 1, 0, -CppAD::sin(X(_theta])), 0, CppAD::cos(X(_theta]));
    ADMatrixXd ADRotZ << CppAD::cos(X(_psi])), -CppAD::sin(X(_psi])), 0, CppAD::sin(X(_psi])), CppAD::cos(X(_psi])), 0, 0, 0, 1;
    ADMatrixXd ADRoti2b = ADRotX * ADRotY * ADRotZ; // Inertial to Body
    ADMatrixXd ADRotb2i = ADRoti2b.transpose();     // Body to Inertial

    // 1. Time derivatives of: xI, yI, zI,
    Xdot.head<3>() = ADRotb2i * Xdot.segment<3>(_U);

    // 2. Time-derivatives of: psi, theta, and phi
    Xdot[_psi] = (X[_Q] * CppAD::sin(X[_phi]) + X[_R] * CppAD::cos(X[_phi])) / cos(X[_theta]);

    Xdot[_theta] = X[_Q] * CppAD::cos(X[_phi]) - X[_R] * CppAD::sin(X[_phi]);

    Xdot[_phi] = X[_P] + (X[_Q] * CppAD::sin(X[_phi]) + X[_R] * CppAD::cos(X[_phi])) * CppAD::tan(X[_theta]);

    // 3. Time-derivatives of : U, V, W
    ADVectorXd transportUVW = X.segment<3>(_P).cross(X.segment<3>(_U)); // [p, q, r] x [u, v, w]
    Xdot[_U] = (-drag(0, 0) * X[_U] - 0.5 * Sgn(ref(_U)) * rho * drag(0, 1) * X[_U] * X[_U]) / mass -
               ((Fg - Fb) / mass) * CppAD::sin(X[_theta]) - transportUVW(1);

    Xdot[_V] = (-drag(1, 0) * X[_V] - 0.5 * Sgn(ref(_V)) * rho * drag(1, 1) * X[_V] * X[_V]) / mass +
               ((Fg - Fb) / mass) * CppAD::cos(X[_theta]) * CppAD::sin(X[_phi]) - transportUVW(2);

    Xdot[_W] = (-drag(2, 0) * X[_W] - 0.5 * Sgn(ref(_W)) * rho * drag(2, 1) * X[_W] * X[_W]) / mass +
               ((Fg - Fb) / mass) * CppAD::cos(X[_theta]) * CppAD::cos(X[_phi]) - transportUVW(3);

    // 4. Time-derivatives of: P, Q, R
    ADVectorXd transportPQR = X.segment<3>(_P).cross(inertia * X.segment<3>(_P)); // [p, q, r] x I*[p, q, r]
    Xdot.segment<3>(_P) = -inertia.inverse() * transportPQR;

    // Compute Jacobian
    vector<double> x;
    for(int i = 0; i < 12; i++)
        x.push_back(ref(i));
    CppAD::ADFun<double> f(X, Xdot);
    jac = f.Jacobian(x);

    Matrix12f A;
    A.setZero();

    // Put jac elements into matrix format
    for(int i = 0; i < 12; i++)
        for(int j = 0; j < 12; j++)
            A(i,j) = (float)jac[i*12 + j];

    return A;
}
