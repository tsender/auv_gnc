#include "auv_gnc/kalman_filter.hpp"

namespace AUV_GNC
{
KalmanFilter::KalmanFilter(const Ref<const MatrixXf> &Ao, const Ref<const MatrixXf> &Ho,
                           const Ref<const MatrixXf> &Qo, const Ref<const MatrixXf> &Ro)
{
    // Verify Parameter Dimensions
    int Arows = Ao.rows(), Acols = Ao.cols(), Hrows = Ho.rows(), Hcols = Ho.cols();
    int Qrows = Qo.rows(), Qcols = Qo.cols(), Rrows = Ro.rows(), Rcols = Ro.cols();

    std::stringstream ss;
    bool throwError = false;
    if (Arows != Acols)
    {
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Ao' of size(" << Arows << "," << Acols << ") is not a square matrix" << std::endl;
        throwError = true;
    }
    if (!Qo.isApprox(Qo.transpose()))
    {
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Qo' of size(" << Qrows << "," << Qcols << ") is not symmetric" << std::endl;
        throwError = true;
    }
    if (!Ro.isApprox(Ro.transpose()))
    {
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Ro' of size(" << Rrows << "," << Rcols << ") is not symmetric" << std::endl;
        throwError = true;
    }
    if (Arows != Hcols)
    {
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Ao' row_size(" << Arows << ") must match param 'Ho' col_size(" << Hcols << ")" << std::endl;
        throwError = true;
    }
    if (Hrows != Rrows)
    {
        ss << "Dimension mismatch in call to KalmanFilter::KalmanFilter(...): Param 'Ho' row_size(" << Hrows << ") must match param 'Ro' row_size(" << Rrows << ")" << std::endl;
        throwError = true;
    }
    if (throwError) // Throw error, notifying user of all errors made
        throw std::runtime_error(ss.str());

    // Resize matrices and initialize
    n_ = Ao.rows();
    m_ = Ho.rows();
    A_ = Ao;
    H_ = Ho;
    Q_ = Qo;
    P_ = Qo; // Initialize error covariance to process covariance
    R_ = Ro;

    K_.resize(n_, m_);
    K_.setZero();
    I_.resize(n_, n_);
    I_.setIdentity();
    Xhat_.resize(n_,1);
    Xhat_.setZero();
    init_ = false;
}

void KalmanFilter::init(const Ref<const VectorXf> &Xo)
{
    // Verify Parameter Dimensions
    int Xorows = Xo.rows();
    if (Xorows != n_)
    {
        std::stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::Init(...): Param 'Xo' row_size(" << Xorows << ") does not match expected row_size(" << n_ << ")" << std::endl;
        throw std::runtime_error(ss.str());
    }

    Xhat_ = Xo;
    init_ = true;
}

// Update Kalman Filter, assuming linear system
VectorXf KalmanFilter::update(const Ref<const VectorXf> &Z)
{
    // Check for initialization of KF
    // If not, default is to leave Xhat as the zero vector
    if (!init_)
        init_ = true;

    // Verify Parameter Dimensions
    int Zrows = Z.rows();
    if (Zrows != m_)
    {
        std::stringstream ss;
        ss << "Dimension mismatch in call to KalmanFilter::Update(...): Param 'Z' row_size(" << Zrows << ") does not match expected row_size(" << m_ << ")" << std::endl;
        throw std::runtime_error(ss.str());
    }

    Xhat_ = A_ * Xhat_;
    P_ = A_ * P_ * A_.transpose() + Q_;
    K_ = P_ * H_.transpose() * ((H_ * P_ * H_.transpose() + R_).inverse());
    Xhat_ = Xhat_ + K_ * (Z - H_ * Xhat_);
    P_ = (I_ - K_ * H_) * P_;
    return Xhat_;
}

// Generic Update - can override state apriori and system matrices
// To be called by an Extended Kalman Filter (EKF)
// A and H matrices are Jacobians calculated by the EKF
// R is a subset of the complete (original) R for the filter
VectorXf KalmanFilter::updateEKF(const Ref<const MatrixXf> &Anew, const Ref<const MatrixXf> &Hnew, const Ref<const MatrixXf> &Rnew,
                                const Ref<const VectorXf> &Xpredict, const Ref<const VectorXf> &Z)
{
    // Verify Parameter Dimensions
    int Xrows = Xpredict.rows(), Zrows = Z.rows();
    int Arows = Anew.rows(), Acols = Anew.cols(), Hrows = Hnew.rows(), Hcols = Hnew.cols(), Rrows = Rnew.rows(), Rcols = Rnew.cols();

    std::stringstream ss;
    bool throwError = false;
    
    if ((Arows != n_) && (Acols != n_))
    {
        ss << "Dimension mismatch in call to KalmanFilter::GenericUpdate(...): Param 'Anew' of size(" << Arows << "," << Acols << ") does not match expected size(" << n_ << "," << n_ << ")" << std::endl;
        throwError = true;
    }
    if (Hcols != n_)
    {
        ss << "Dimension mismatch in call to KalmanFilter::GenericUpdate(...): Param 'Hnew' of col_size(" << Hcols << ") does not match expected col_size(" << n_ << ")" << std::endl;
        throwError = true;
    }
    if (!Rnew.isApprox(Rnew.transpose()))
    {
        ss << "Dimension mismatch in call to KalmanFilter::GenericUpdate(...): Param 'Rnew' of size(" << Rrows << "," << Rcols << ") is not symmetric" << std::endl;
        throwError = true;
    }
    if (Rrows != Hrows)
    {
        ss << "Dimension mismatch in call to KalmanFilter::GenericUpdate(...): Param 'Hnew' of row_size(" << Hrows << ") does not match expected param 'Rnew' row_size(" << Rrows << ")" << std::endl;
        throwError = true;
    }
    if (Xrows != n_)
    {
        ss << "Dimension mismatch in call to KalmanFilter::GenericUpdate(...): Param 'Xpredict' row_size(" << Xrows << ") does not match expected row_size(" << n_ << ")" << std::endl;
        throwError = true;
    }
    if (Zrows != Hrows)
    {
        ss << "Dimension mismatch in call to KalmanFilter::GenericUpdate(...): Param 'Z' row_size(" << Zrows << ") does not match param 'Hnew' row_size(" << Hrows << ")" << std::endl;
        throwError = true;
    }
    if (throwError) // Throw error, notifying user of all errors made
        throw std::runtime_error(ss.str());

    A_.resize(Arows, Acols);
    H_.resize(Hrows, Hcols);
    R_.resize(Rrows, Rcols);
    A_ = Anew;
    H_ = Hnew;
    R_ = Rnew;
    P_ = A_ * P_ * A_.transpose() + Q_;
    K_ = P_ * H_.transpose() * ((H_ * P_ * H_.transpose() + R_).inverse());
    Xhat_ = Xpredict + K_ * (Z - H_ * Xpredict);
    P_ = (I_ - K_ * H_) * P_;

    return Xhat_;
}

VectorXf KalmanFilter::getXhat()
{
    return Xhat_;
}

MatrixXf KalmanFilter::getErrorCovariance()
{
    return P_;
}
}