#include "auv_guidance/basic_trajectory.hpp"

namespace auv_guidance
{
/**
 * @param start Starting waypoint
 * @param end Ending waypoint
 * @param travelDuration Desired travel duration [s]
 */
BasicTrajectory::BasicTrajectory(Waypoint *wStart, Waypoint *wEnd, TGenLimits *tGenLimits)
{
    wStart_ = wStart;
    wEnd_ = wEnd;
    tGenLimits_ = tGenLimits;

    qStop_.setIdentity();
    deltaVec_.setZero();
    unitVec_.setZero();
    maxVelocityVec_.setZero();

    maxVelocity_ = 0;
    stopDuration_ = 0;
    simultaneousDuration_ = 0;
    longDuration_ = 0;
    distance_ = 0;

    longTrajectory_ = false;
    simultaneousTrajectory_ = true;
    exceedsMaxSpeed_ = false;

    BasicTrajectory::setStopTrajectory();
    BasicTrajectory::setPrimaryTrajectory();
}

void BasicTrajectory::setStopTrajectory()
{
    // Get stop position
    double transVel = wStart_->velI().norm();
    double transAccel = wStart_->accelI().norm();
    double accel = 0;
    if (fabs(wStart_->velI()(0)) > fabs(wStart_->velI()(1))) // Xvel > Yvel
    {
        accel = tGenLimits_->maxXAccel();
        if (fabs(wStart_->velI()(2)) > fabs(wStart_->velI()(0))) // Zvel > Xvel
            accel = tGenLimits_->maxZAccel();
    }
    else
    {
        accel = tGenLimits_->maxYAccel();
        if (fabs(wStart_->velI()(2)) > fabs(wStart_->velI()(1))) // Zvel > Yvel
            accel = tGenLimits_->maxZAccel();
    }
    double distance = (2.0 / 3.0) * (transVel * transVel) / accel;
    Eigen::Vector3d restDeltaVec = wStart_->velI().normalized() * distance;
    Eigen::Vector3d stopPos = wStart_->posI() + restDeltaVec;

    // Get stop quaternion
    Eigen::Vector4d angleAxis = Eigen::Vector4d::Zero();
    angleAxis.tail<3>() = wStart_->angVelB().normalized();
    double angVel = wStart_->angVelB().norm(); // Magnitude
    double angularDistance = (2.0 / 3.0) * (angVel * angVel) / tGenLimits_->maxRotAccel();
    angleAxis(0) = angularDistance;

    Eigen::Quaterniond qRotate = auv_core::rot3d::angleAxis2Quat(angleAxis); // Relative to B-frame
    qStop_ = wStart_->quaternion() * qRotate;                                         // Apply qRotate FROM qStart

    Eigen::Vector3d zero3d = Eigen::Vector3d::Zero();
    wStop_ = new Waypoint(stopPos, zero3d, zero3d, qStop_, zero3d);

    // Find travel time for translation and rotation, take the longer one
    Eigen::Vector4d transStart = Eigen::Vector4d::Zero();
    Eigen::Vector4d transEnd = Eigen::Vector4d::Zero();
    Eigen::Vector4d rotStart = Eigen::Vector4d::Zero();
    Eigen::Vector4d rotEnd = Eigen::Vector4d::Zero();

    transStart << 0, transVel, transAccel, tGenLimits_->xyzJerk(distance);
    transEnd << distance, 0, 0, tGenLimits_->xyzJerk(distance);
    rotStart << 0, angVel, 0, tGenLimits_->rotJerk(angularDistance);
    rotEnd << angularDistance, 0, 0, tGenLimits_->rotJerk(angularDistance);

    double timeTrans = 0, timeRot = 0;
    MinJerkTimeSolver *mjts;
    mjts = new MinJerkTimeSolver(transStart, transEnd);
    timeTrans = mjts->getTime();
    mjts = new MinJerkTimeSolver(rotStart, rotEnd);
    timeRot = mjts->getTime();
    stopDuration_ = std::max(timeTrans, timeRot); // Take longer duration

    stStop_ = new SimultaneousTrajectory(wStart_, wStop_, stopDuration_);
    totalDuration_ = stopDuration_;
}

void BasicTrajectory::computeMaxVelocity()
{
    deltaVec_ = wEnd_->posI() - wStop_->posI();
    unitVec_ = deltaVec_.normalized();

    double distanceXY = deltaVec_.head<2>().norm();
    double distanceZ = fabs(deltaVec_(2));
    distance_ = deltaVec_.norm();

    if (distanceXY > tGenLimits_->maxXYDistance())
    {
        simultaneousTrajectory_ = false;
        std::cout << "BT: dixtance XY too large: " << distanceXY << " > " << tGenLimits_->maxXYDistance() << std::endl; // Debug
    }
    if (distanceZ > tGenLimits_->maxZDistance())
    {
        simultaneousTrajectory_ = false;
        std::cout << "BT: dixtance Z too large: " << distanceZ << " > " << tGenLimits_->maxZDistance() << std::endl; // Debug
    }

    BasicTrajectory::computeSimultaneousTime();

    // Find max translational velocity
    Eigen::Vector3d transStart = Eigen::Vector3d::Zero();
    Eigen::Vector3d transEnd = Eigen::Vector3d::Zero();
    transEnd(0) = distance_;
    mjtHelper_ = new MinJerkTrajectory(transStart, transEnd, simultaneousDuration_);

    initialMaxVelocity_ = fabs(mjtHelper_->getMiddleVelocity());
    maxVelocity_ = initialMaxVelocity_;
    maxVelocityVec_ = unitVec_ * maxVelocity_;
}

void BasicTrajectory::computeSimultaneousTime()
{
    // Translation
    Eigen::Vector4d transStart = Eigen::Vector4d::Zero();
    Eigen::Vector4d transEnd = Eigen::Vector4d::Zero();
    transStart << 0, 0, 0, tGenLimits_->xyzJerk(distance_);
    transEnd << distance_, 0, 0, tGenLimits_->xyzJerk(distance_);

    // Rotation
    qEnd_ = wEnd_->quaternion();
    Eigen::Quaterniond qDiff = qStop_.conjugate() * qEnd_; // Error quaternion wrt B-frame (q2 * q1.conjugate is wrt I-frame)
    double angularDistance = auv_core::rot3d::quat2AngleAxis(qDiff)(0);

    Eigen::Vector4d rotStart = Eigen::Vector4d::Zero();
    Eigen::Vector4d rotEnd = Eigen::Vector4d::Zero();
    rotStart << 0, 0, 0, tGenLimits_->rotJerk(angularDistance);
    rotEnd << angularDistance, 0, 0, tGenLimits_->rotJerk(angularDistance);

    // Compute durations
    double timeTrans = 0, timeRot = 0;
    MinJerkTimeSolver *mjts;
    mjts = new MinJerkTimeSolver(transStart, transEnd);
    timeTrans = mjts->getTime();
    mjts = new MinJerkTimeSolver(rotStart, rotEnd);
    timeRot = mjts->getTime();

    simultaneousDuration_ = std::max(timeTrans, timeRot); // Take longer duration
    std::cout << "BT: simultaneous duration: " << simultaneousDuration_ << std::endl; // Debug
}

void BasicTrajectory::setPrimaryTrajectory()
{
    BasicTrajectory::computeMaxVelocity();

    double maxXYVel = maxVelocityVec_.head<2>().norm();
    double maxZVel = fabs(maxVelocityVec_(2));

    // Verify XYZ velocities are not violated
    if (maxXYVel > tGenLimits_->maxXVel())
    {
        simultaneousTrajectory_ = false;
        std::cout << "BT: max XY velocity too large: " << maxXYVel <<  " > " << tGenLimits_->maxXVel() << std::endl; // Debug
        maxXYVel = tGenLimits_->maxXVel();
    }
    if (maxZVel > tGenLimits_->maxZVel())
    {
        simultaneousTrajectory_ = false;
        std::cout << "BT: max Z velocity too large: " << maxZVel << " > " << tGenLimits_->maxZVel() << std::endl; // Debug
        maxZVel = tGenLimits_->maxZVel();
    }

    // Update max velocity vector
    maxVelocity_ = sqrt(maxXYVel * maxXYVel + maxZVel * maxZVel);
    maxVelocityVec_ = unitVec_ * maxVelocity_;

    if (!simultaneousTrajectory_) // Execute long trajectory
    {
        longTrajectory_ = true;
        double cruiseRatio = 1.0 - maxVelocity_ / initialMaxVelocity_;
        ltPrimary_ = new LongTrajectory(wStop_, wEnd_, tGenLimits_, cruiseRatio, maxVelocity_);
        longDuration_ = ltPrimary_->getTime();
        totalDuration_ += longDuration_;
    }
    else // Execute simultaneous trajectory
    {
        simultaneousTrajectory_ = true;
        stPrimary_ = new SimultaneousTrajectory(wStop_, wEnd_, simultaneousDuration_);
        totalDuration_ = simultaneousDuration_;
    }
    std::cout << "BT long trajectory " << longTrajectory_ << std::endl; // Debug
    std::cout << "BT simultaneous trajectory " << simultaneousTrajectory_ << std::endl; // Debug
}

double BasicTrajectory::getTime()
{
    return totalDuration_;
}

/**
 * @param time Time to compute the state at
 * Computes the trajectory state at the specified time
 */
auv_core::Vector13d BasicTrajectory::computeState(double time)
{
    if (time <= stopDuration_)
    {
        //std::cout << "ST: stop trajectory at time " << time << std::endl;
        return stStop_->computeState(time);
    }
    else if (simultaneousTrajectory_)
    {
        //std::cout << "ST: main trajectory at time " << time - stopDuration_ << std::endl;
        return stPrimary_->computeState(time - stopDuration_);
    }
    else if (longTrajectory_)
    {
        return ltPrimary_->computeState(time - stopDuration_);
    }
}

/**
 * @param time Time to compute accelerations at
 * Compute inertial translational acceleration and time-derivative of angular veocity, 
 * both expressed in B-frame, at specified time
 */
auv_core::Vector6d BasicTrajectory::computeAccel(double time)
{
    if (time <= stopDuration_)
    {
        return stStop_->computeAccel(time);
    }
    else if (simultaneousTrajectory_)
    {
        return stPrimary_->computeAccel(time - stopDuration_);
    }
    else if (longTrajectory_)
    {
        return ltPrimary_->computeAccel(time - stopDuration_);
    }
}
} // namespace auv_guidance