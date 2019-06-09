#include "auv_guidance/short_range_trajectory.hpp"

namespace AUVGuidance
{
/**
 * @param start Starting waypoint
 * @param end Ending waypoint
 * @param travelDuration Desired travel duration [s]
 */
ShortRangeTrajectory::ShortRangeTrajectory(Waypoint *start, Waypoint *end, double travelDuration)
{
    wStart_ = start;
    wEnd_ = end;
    travelDuration_ = travelDuration;

    qDiff_.w() = 1;
    qDiff_.x() = 0;
    qDiff_.y() = 0;
    qDiff_.z() = 0;
    
    xState_.setZero();
    yState_.setZero();
    zState_.setZero();
    angleState_.setZero();
    rotationAxis_.setZero();

    ShortRangeTrajectory::initTrajectory();
}

/**
 * Initialize the min jerk trajectories
 */
void ShortRangeTrajectory::initTrajectory()
{
    qStart_ = wStart_->quaternion().normalized();
    qEnd_ = wEnd_->quaternion().normalized();
    qDiff_ = qStart_.conjugate() * qEnd_; // Error quaternion wrt B-frame (q2 * q1.conjugate is opposite)
    
    Vector4d angleAxis = AUVMathLib::quaternion2AngleAxis(qDiff_);
    angularDistance_ = angleAxis(0);
    rotationAxis_ = angleAxis.tail<3>(); // Get axis relative to Body-frame

    Vector3d angleStart, angleEnd;
    angleStart.setZero(), angleEnd.setZero();
    angleEnd(1) = angularDistance_;

    mjtX_ = new MinJerkTrajectory(wStart_->xI(), wEnd_->xI(), travelDuration_);
    mjtY_ = new MinJerkTrajectory(wStart_->yI(), wEnd_->yI(), travelDuration_);
    mjtZ_ = new MinJerkTrajectory(wStart_->zI(), wEnd_->zI(), travelDuration_);
    mjtAtt_ = new MinJerkTrajectory(angleStart, angleEnd, travelDuration_);
}

/**
 * @param time Time to compute the state at
 * Computes the trajectory state at the specified time
 */
Vector12d ShortRangeTrajectory::computeState(double time)
{
    Vector12d state;
    state.setZero();

    xState_ = mjtX_->computeState(time);
    yState_ = mjtY_->computeState(time);
    zState_ = mjtZ_->computeState(time);
    angleState_ = mjtAtt_->computeState(time);
    
    // Translational Components
    Vector3d xyz, uvw;
    xyz.setZero();
    uvw.setZero();

    // Inertial position expressed in I-frame
    xyz(0) = xState_(0);
    xyz(1) = yState_(0);
    xyz(2) = zState_(0); 

    // Inertial velocity expressed in I-frame
    uvw(0) = xState_(1);
    uvw(1) = yState_(1);
    uvw(2) = zState_(1); 

    // Rotational Components
    Vector3d pqr = Vector3d::Zero();
    Vector4d q = Vector4d::Zero();

    if (time >= 0 && time <= travelDuration_)
    {
        double frac = time / travelDuration_;
        qSlerp_ = qStart_.slerp(frac, qEnd_); // Attitude wrt I-frame
        qSlerp_ = qSlerp_.normalized();
    }
    else if (time < 0)
    {
        qSlerp_ = qStart_;
    }
    else if (time > travelDuration_)
    {
        qSlerp_ = qEnd_;
    }
    
    uvw = qSlerp_.conjugate() * uvw; // Inertial velocity expressed in B-frame

    pqr = rotationAxis_ * angleState_(0); // Angular velocity expressed in B-frame
    q(0) = qSlerp_.w();
    q(1) = qSlerp_.x();
    q(2) = qSlerp_.y();
    q(3) = qSlerp_.z();

    state.segment<3>(AUVControl::AUVModel::xI_) = xyz;
    state.segment<3>(AUVControl::AUVModel::U_) = uvw;
    state.segment<3>(AUVControl::AUVModel::q1_) = q.tail<3>();
    state.segment<3>(AUVControl::AUVModel::P_) = pqr;
    return state;
}

/**
 * @param time Time to compute accelerations at
 * Compute inertial translational acceleration and time-derivative of angular veocity, 
 * both expressed in B-frame, at specified time
 */
Vector6d ShortRangeTrajectory::computeAccel(double time)
{
    Vector6d accel;
    accel.setZero();

    Vector3d inertialTransAccel = Vector3d::Zero();
    inertialTransAccel(0) = xState_(2);
    inertialTransAccel(1) = yState_(2);
    inertialTransAccel(2) = zState_(2);
    inertialTransAccel = qSlerp_.conjugate() * inertialTransAccel;

    Vector3d pqrDot = Vector3d::Zero();
    pqrDot = rotationAxis_ * angleState_(1);

    accel << inertialTransAccel, pqrDot;
    return accel;
}
} // namespace AUVGuidance