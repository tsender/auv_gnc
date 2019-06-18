#include "auv_guidance/simultaneous_trajectory.hpp"

namespace AUVGuidance
{
/**
 * @param start Starting waypoint
 * @param end Ending waypoint
 * @param travelDuration Desired travel duration [s]
 */
SimultaneousTrajectory::SimultaneousTrajectory(Waypoint *start, Waypoint *end, double duration)
{
    wStart_ = start;
    wEnd_ = end;
    totalDuration_ = duration;

    qDiff_.w() = 1;
    qDiff_.x() = 0;
    qDiff_.y() = 0;
    qDiff_.z() = 0;
    
    xState_.setZero();
    yState_.setZero();
    zState_.setZero();
    angleState_.setZero();
    rotationAxis_.setZero();

    SimultaneousTrajectory::initTrajectory();
}

/**
 * Initialize the min jerk trajectories
 */
void SimultaneousTrajectory::initTrajectory()
{
    qStart_ = wStart_->quaternion().normalized();
    qEnd_ = wEnd_->quaternion().normalized();
    qDiff_ = qStart_.conjugate() * qEnd_; // Error quaternion wrt B-frame (q2 * q1.conjugate is wrt I-frame)
    
    Eigen::Vector4d angleAxis = AUVMathLib::quaternion2AngleAxis(qDiff_);
    angularDistance_ = angleAxis(0);
    double angVel = wStart_->angVelB().squaredNorm();
    rotationAxis_ = angleAxis.tail<3>(); // Get axis relative to Body-frame

    Eigen::Vector3d angleStart = Eigen::Vector3d::Zero(); 
    Eigen::Vector3d angleEnd = Eigen::Vector3d::Zero();
    angleStart(1) = angVel;
    angleEnd(0) = angularDistance_;

    mjtX_ = new MinJerkTrajectory(wStart_->xI(), wEnd_->xI(), totalDuration_);
    mjtY_ = new MinJerkTrajectory(wStart_->yI(), wEnd_->yI(), totalDuration_);
    mjtZ_ = new MinJerkTrajectory(wStart_->zI(), wEnd_->zI(), totalDuration_);
    mjtAtt_ = new MinJerkTrajectory(angleStart, angleEnd, totalDuration_);
}

double SimultaneousTrajectory::getTime()
{
    return totalDuration_;
}

/**
 * @param time Time to compute the state at
 * Computes the trajectory state at the specified time
 */
Vector12d SimultaneousTrajectory::computeState(double time)
{
    Vector12d state;
    state.setZero();

    xState_ = mjtX_->computeState(time);
    yState_ = mjtY_->computeState(time);
    zState_ = mjtZ_->computeState(time);
    angleState_ = mjtAtt_->computeState(time);
    
    // Translational Components
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d uvw = Eigen::Vector3d::Zero();

    // Inertial position expressed in I-frame
    xyz(0) = xState_(0);
    xyz(1) = yState_(0);
    xyz(2) = zState_(0); 

    // Inertial velocity expressed in I-frame
    uvw(0) = xState_(1);
    uvw(1) = yState_(1);
    uvw(2) = zState_(1); 

    // Rotational Components
    Eigen::Vector3d pqr = Eigen::Vector3d::Zero();
    Eigen::Vector4d q = Eigen::Vector4d::Zero();

    if (time >= 0 && time <= totalDuration_)
    {
        double frac = time / totalDuration_;
        qSlerp_ = qStart_.slerp(frac, qEnd_); // Attitude wrt I-frame
        qSlerp_ = qSlerp_.normalized();
    }
    else if (time < 0)
    {
        qSlerp_ = qStart_;
    }
    else if (time > totalDuration_)
    {
        qSlerp_ = qEnd_;
    }
    
    uvw = qSlerp_.conjugate() * uvw; // Inertial velocity expressed in B-frame

    pqr = rotationAxis_ * angleState_(0); // Angular velocity expressed in B-frame
    q(0) = qSlerp_.w();
    q(1) = qSlerp_.x();
    q(2) = qSlerp_.y();
    q(3) = qSlerp_.z();

    state.segment<3>(AUVControl::AUVModel::STATE_XI) = xyz;
    state.segment<3>(AUVControl::AUVModel::STATE_U) = uvw;
    state.segment<3>(AUVControl::AUVModel::STATE_Q1) = q.tail<3>();
    state.segment<3>(AUVControl::AUVModel::STATE_P) = pqr;
    return state;
}

/**
 * @param time Time to compute accelerations at
 * Compute inertial translational acceleration and time-derivative of angular veocity, 
 * both expressed in B-frame, at specified time
 */
Vector6d SimultaneousTrajectory::computeAccel(double time)
{
    Vector6d accel;
    accel.setZero();

    Eigen::Vector3d inertialTransAccel = Eigen::Vector3d::Zero();
    inertialTransAccel(0) = xState_(2);
    inertialTransAccel(1) = yState_(2);
    inertialTransAccel(2) = zState_(2);
    inertialTransAccel = qSlerp_.conjugate() * inertialTransAccel;

    Eigen::Vector3d pqrDot = Eigen::Vector3d::Zero();
    pqrDot = rotationAxis_ * angleState_(1);

    accel << inertialTransAccel, pqrDot; // Both expressed in B-frame
    return accel;
}
} // namespace AUVGuidance