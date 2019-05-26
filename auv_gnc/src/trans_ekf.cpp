#include "riptide_gnc/trans_ekf.h"

// NOTE: pos sensors are Inertial-FRAME, and vel and accel sensors are Body-FRAME
TransEKF::TransEKF(const Ref<const Matrix3i> &fullMsmtMaskIn, const Ref<const MatrixXf> &RposIn, const Ref<const MatrixXf> &RvelIn,
                   const Ref<const MatrixXf> &RaccelIn, const Ref<const Matrix9f> &Qin)
{
    n = 9;
    init = false;
    Xhat.setZero();
    fullMsmtMask = fullMsmtMaskIn; // Mask of all measurements that the provided sensors measure
    Rpos = RposIn;
    Rvel = RvelIn;
    Raccel = RaccelIn;
    Q = Qin;

    // Initialize the EKF using generic matrices, as they will be overridden with each update call
    Matrix9f A;
    A.setIdentity();

    int m = fullMsmtMask.sum();
    MatrixXf H(m,n);
    H.setZero();

    MatrixXf R(m,m);
    R.setIdentity();

    EKF = new KalmanFilter(A, H, Q, R);
}

void TransEKF::Init(const Ref<const VectorXf>& Xo)
{
    // Verify Parameter Dimensions
    int Xorows = Xo.rows();
    if (Xorows != n)
    {
        stringstream ss;
        ss << "Dimension mismatch in call to TransEKF::Init(...): Param 'Xo' row_size(" << Xorows << ") does not match expected row_size(" << n << ")" << endl;
        throw std::runtime_error(ss.str());
    }

    Xhat = Xo;
    init = true;
}

// Inertial Measurement Update - sensor readings are from inertial sensors
// dt = time step [s] since last call of this update function
// attitude = Euler Angles in the order of (yaw, pitch, roll)
// sensorMask = indicates if the sensor (pos, vel, and/or accel) has new data
// Z = the actual sensor data, same format as Zmask
VectorXf TransEKF::Update(float dt, const Ref<const Vector3f> &attitude, const Ref<const Vector3i> &sensorMask, const Ref<const Matrix3f> &Zmat)
{
    // Check for initialization of KF
    // If not, default is to leave Xhat as the zero vector
    if (!init)
        init = true;
    
    // Create A (state transition matrix) 
    // Using kinematic relationships in a constant acceleration model
    Matrix9f A;
    A.setIdentity();

    Vector3f dtVector;
    dtVector << dt, dt, dt;
    Matrix3f dtMat = diag(dtVector); // Diagonal matrix of dt

    // Rotation matrix from B-frame to I-frame
    Matrix3f Rotb2i = AUVMathLib::GetEulerRotMat(attitude).transpose();

    // Constant Acceleration model:
    // x = x_prev + dt*v + (0.5*dt^2)*a
    // v = v_prev + dt*a
    // a = a_prev
    A.block<3, 3>(0, 3) = dt * Rotb2i;
    A.block<3, 3>(0, 6) = 0.5 * pow(dt, 2) * Rotb2i;
    A.block<3, 3>(3, 6) = dtMat;

    // Get mask for which data fields are present
    // diag(sensorMask) acts as a filter on fullmsmtMask to provide the dataMask
    Matrix3i dataMask = fullMsmtMask * diag(sensorMask);
    int m = dataMask.sum(); // Number of msmts in this iteration
    
    // Create H (observation/measurement matrix) and Z (measurement vector)
    MatrixXf H(m, n);
    VectorXf Z(m, 1);
    H.setZero();
    Z.setZero();
    int i = 0;

    // Fill in H matrix based on dataMask
    for (int k = 0; k < 3; j++) // Traverse across cols
    {
        for (int j = 0; j < 3; k++) // Traverse down rows
        {
            if(dataMask(j,k))
            {
                H(i, (3*k + j) = 1;
                Z(i++) = Zmat(j,k);
            }
        }
    }
    
    // Create R (measurement noise covariance matrix) with help from dataMask
    MatrixXf R(m,m);
    R.setZero();
    int p = dataMask.col(0).sum();
    int v = dataMask.col(1).sum();
    int a = dataMask.col(2).sum();
    if (p > 0)
        R.block<p,p>(0,0) = Rpos;
    if (v > 0)
        R.block<v,v>(p,p) = Rvel;
    if (a > 0)
        R.block<a,a>(p+v, p+v) = Raccel;

    // Populate Xpredict vector
    Vector9f Xpredict = A*Xhat;

    // Run update step
    Xhat = EKF->EKFUpdate(A, H, R, Xpredict, Z);
    return Xhat;
}