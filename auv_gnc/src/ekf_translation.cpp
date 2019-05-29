#include "auv_gnc/ekf_translation.hpp"

namespace AUV_GNC
{
// NOTE: pos sensors are Inertial-FRAME, and vel and accel sensors are Body-FRAME
EKFTranslation::EKFTranslation(const Ref<const Matrix3i> &fullMsmtMaskIn, const Ref<const MatrixXf> &RposIn, const Ref<const MatrixXf> &RvelIn,
                               const Ref<const MatrixXf> &RaccelIn, const Ref<const Matrix9f> &Qin)
{
    n_ = 9;
    init_ = false;
    Xhat_.setZero();
    fullMsmtMask_ = fullMsmtMaskIn; // Mask of all measurements that the provided sensors measure
    Rpos_ = RposIn;
    Rvel_ = RvelIn;
    Raccel_ = RaccelIn;
    Q_ = Qin;

    // Initialize the EKF using generic matrices, as they will be overridden with each update call
    Matrix9f A;
    A.setIdentity();

    int m = fullMsmtMask_.sum();
    MatrixXf H(m, n_);
    H.setZero();

    MatrixXf R(m, m);
    R.setIdentity();

    ekf_ = new KalmanFilter(A, H, Q_, R);
}

void EKFTranslation::init(const Ref<const VectorXf> &Xo)
{
    // Verify Parameter Dimensions
    int Xorows = Xo.rows();
    if (Xorows != n_)
    {
        std::stringstream ss;
        ss << "Dimension mismatch in call to EKFTranslation::Init(...): Param 'Xo' row_size(" << Xorows << ") does not match expected row_size(" << n_ << ")" << std::endl;
        throw std::runtime_error(ss.str());
    }

    Xhat_ = Xo;
    init_ = true;
}

// Inertial Measurement Update - sensor readings are from inertial sensors
// dt = time step [s] since last call of this update function
// attitude = Euler Angles in the order of (yaw, pitch, roll)
// sensorMask = indicates if the sensor (pos, vel, and/or accel) has new data
// Z = the actual sensor data, same format as Zmask
VectorXf EKFTranslation::update(float dt, const Ref<const Vector3f> &attitude, const Ref<const Vector3i> &sensorMask, const Ref<const Matrix3f> &Zmat)
{
    // Check for initialization of KF
    // If not, default is to leave Xhat as the zero vector
    if (!init_)
        init_ = true;

    // Create A (state transition matrix)
    // Using kinematic relationships in a constant acceleration model
    Matrix9f A;
    A.setIdentity();

    Vector3f dtVector;
    dtVector << dt, dt, dt;
    Matrix3f dtMat = dtVector.asDiagonal(); // Diagonal matrix of dt

    // Rotation matrix from B-frame to I-frame
    Matrix3f Rotb2i = AUVMathLib::getEulerRotationMat(attitude).transpose();

    // Constant Acceleration model:
    // x = x_prev + dt*v + (0.5*dt^2)*a
    // v = v_prev + dt*a
    // a = a_prev
    A.block<3, 3>(0, 3) = dt * Rotb2i;
    A.block<3, 3>(0, 6) = 0.5 * pow(dt, 2) * Rotb2i;
    A.block<3, 3>(3, 6) = dtMat;

    // Get mask for which data fields are present
    // diag(sensorMask) acts as a filter on fullmsmtMask to provide the dataMask
    Matrix3i dataMask = fullMsmtMask_ * sensorMask.asDiagonal();
    int m = dataMask.sum(); // Number of msmts in this iteration

    // Create H (observation/measurement matrix) and Z (measurement vector)
    MatrixXf H(m, n_);
    VectorXf Z(m, 1);
    H.setZero();
    Z.setZero();
    int i = 0;

    // Fill in H matrix based on dataMask, fill in Z vector
    for (int k = 0; k < 3; k++) // Traverse across cols
    {
        for (int j = 0; j < 3; j++) // Traverse down rows
        {
            if (dataMask(j, k))
            {
                H(i, (3 * k + j)) = 1;
                Z(i++) = Zmat(j, k);
            }
        }
    }

    // Create R (measurement noise covariance matrix) with help from dataMask
    MatrixXf R(m, m);
    R.setZero();
    int p = dataMask.col(0).sum();
    int v = dataMask.col(1).sum();
    int a = dataMask.col(2).sum();
    if (p > 0)
        R.block(0, 0, p, p) = Rpos_;
    if (v > 0)
        R.block(p, p, v, v) = Rvel_;
    if (a > 0)
        R.block(p + v, p + v, a, a) = Raccel_;

    // Populate Xpredict vector
    Vector9f Xpredict = A * Xhat_;

    // Run update step
    Xhat_ = ekf_->updateEKF(A, H, R, Xpredict, Z);
    return Xhat_;
}
} // namespace AUV_GNC