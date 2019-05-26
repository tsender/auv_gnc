#include "riptide_gnc/kalman_filter.h"

KalmanFilter::KalmanFilter(const Ref<const MatrixXf> &Ao, const Ref<const MatrixXf> &Ho,
                           const Ref<const MatrixXf> &Qo, const Ref<const MatrixXf> &Ro)
{
    // Verify Parameter Dimensions
    int Arows = Ao.rows(), Acols = Ao.cols(), Hrows = Ho.rows(), Hcols = Ho.cols();
    int Qrows = Qo.rows(), Qcols = Qo.cols(), Rrows = Ro.rows(), Rcols = Ro.cols();

    stringstream ss;
    bool throw_error = false;
    if (Arows != Acols)
    {
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Ao' of size(" << Arows << "," << Acols << ") is not a square matrix" << endl;
        throw_error = true;
    }
    if (!Qo.isApprox(Qo.transpose()))
    {
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Qo' of size(" << Qrows << "," Qcols << ") is not symmetric" << endl;
        throw_error = true;
    }
    if (!Ro.isApprox(Ro.transpose()))
    {
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Ro' of size(" << Rrows << "," Rcols << ") is not symmetric" << endl;
        throw_error = true;
    }
    if (Arows != Hcols)
    {
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Ao' row_size(" << Arows << ") must match param 'Ho' col_size(" << Hcols << ")" << endl;
        throw_error = true;
    }
    if (Hrows != Rrows)
    {
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Ho' row_size(" << Hrows << ") must match param 'Ro' row_size(" << Rrows << ")" << endl;
        throw_error = true;
    }
    if (throw_error) // Throw error, notifying user of all errors made
        throw std::runtime_error(ss.str());

    // Resize matrices and initialize
    n = Ao.rows();
    m = Ho.rows();
    A = Ao;
    H = Ho;
    Q = Qo;
    P = Qo; // Initialize error covariance to process covariance
    R = Ro;

    K.resize(n, m);
    K.setZero();
    I.resize(n, n);
    I.setIdentity();
    Xhat.resize(n,1);
    Xhat.setZero();
    init = false;
}

void KalmanFilter::Init(const Ref<const VectorXf> &Xo)
{
    // Verify Parameter Dimensions
    int Xorows = Xo.rows();
    if (Xorows != n)
    {
        stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::Init(...): Param 'Xo' row_size(" << Xorows << ") does not match expected row_size(" << n << ")" << endl;
        throw std::runtime_error(ss.str());
    }

    Xhat = Xo;
    init = true;
}

// Update Kalman Filter, assuming linear system
VectorXf KalmanFilter::Update(const Ref<const VectorXf> &Z)
{
    // Check for initialization of KF
    // If not, default is to leave Xhat as the zero vector
    if (!init)
        init = true;

    // Verify Parameter Dimensions
    int Zrows = Z.rows();
    if (Zrows != m)
    {
        stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::Update(...): Param 'Z' row_size(" << Zrows << ") does not match expected row_size(" << m << ")" << endl;
        throw std::runtime_error(ss.str());
    }

    

    Xhat = A * Xhat;
    P = A * P * A.transpose() + Q;
    K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
    Xhat = Xhat + K * (Z - H * Xhat);
    P = (I - K * H) * P;
    return Xhat;
}

// Generic Update - can override state apriori and system matrices
// To be called by an Extended Kalman Filter (EKF)
// A and H matrices are Jacobians calculated by the EKF
// R is a subset of the complete (original) R for the filter
VectorXf KalmanFilter::EKFUpdate(const Ref<const MatrixXf> &Anew, const Ref<const MatrixXf> &Hnew, const Ref<const MatrixXf> &Rnew,
                                const Ref<const VectorXf> &Xpredict, const Ref<const VectorXf> &Z)
{
    // Verify Parameter Dimensions
    int Xrows = Xpredict.rows(), Zrows = Z.rows();
    int Arows = Anew.rows(), Acols = Anew.cols(), Hrows = Hnew.rows(), Hcols = Hnew.cols(), Rrows = Rnew.rows(), Rcols = Rnew.cols();

    stringstream ss;
    bool throw_error = false;
    
    if ((Arows != n) && (Acols != n)
    {
        ss << "Dimension mismatch in call to KalmanFilter::GenericUpdate(...): Param 'Anew' of size(" << Arows << "," << Acols << ") does not match expected size(" << n << "," << n << ")" << endl;
        throw_error = true;
    }
    if (Hcols != n)
    {
        ss << "Dimension mismatch in call to KalmanFilter::GenericUpdate(...): Param 'Hnew' of col_size(" << Hcols << ") does not match expected col_size(" << n << ")" << endl;
        throw_error = true;
    }
    if (!Rnew.isApprox(Rnew.transpose()))
    {
        ss << "Dimension mismatch in call to KalmanFilter::GenericUpdate(...): Param 'Rnew' of size(" << Rrows << "," Rcols << ") is not symmetric" << endl;
        throw_error = true;
    }
    if (Rrows != Hrows)
    {
        ss << "Dimension mismatch in call to KalmanFilter::GenericUpdate(...): Param 'Hnew' of row_size(" << Hrows << ") does not match expected param 'Rnew' row_size(" << Rrows << ")" << endl;
        throw_error = true;
    }
    if (Xrows != n)
    {
        ss << "Dimension mismatch in call to KalmanFilter::GenericUpdate(...): Param 'Xpredict' row_size(" << Xrows << ") does not match expected row_size(" << n ")" << endl;
        throw_error = true;
    }
    if (Zrows != Hrows)
    {
        ss << "Dimension mismatch in call to KalmanFilter::GenericUpdate(...): Param 'Z' row_size(" << Zrows << ") does not match param 'Hnew' row_size(" << Hrows << ")" << endl;
        throw_error = true;
    }
    if (throw_error) // Throw error, notifying user of all errors made
        throw std::runtime_error(ss.str());

    A.resize(Arows, Acols);
    H.resize(Hrows, Hcols);
    R.resize(Rrows, Rcols);
    A = Anew;
    H = Hnew;
    R = Rnew;
    P = A * P * A.transpose() + Q;
    K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
    Xhat = Xpredict + K * (Z - H * Xpredict);
    P = (I - K * H) * P;

    return Xhat;
}

VectorXf KalmanFilter::GetXhat()
{
    return Xhat;
}

MatrixXf KalmanFilter::GetErrorCovariance()
{
    return P;
}