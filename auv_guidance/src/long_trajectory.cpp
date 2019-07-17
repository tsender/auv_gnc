#include "auv_guidance/long_trajectory.hpp"

namespace auv_guidance
{
/**
 * @param start Starting waypoint
 * @param end Ending waypoint
 * @param rotationDuration Desired rotation duration [s]
 * @param accelDuration Desired acceleration duration [s]
 * @param cruiseRatio Indicates what fraction of the total distance is to be traveled while cruising
 * @param cruiseSpeed Vehicle speed while cruising
 */
LongTrajectory::LongTrajectory(Waypoint *wStart, Waypoint *wEnd, TGenLimits *tGenLimits, double cruiseRatio, double cruiseSpeed)
{
    wStart_ = wStart;
    wEnd_ = wEnd;
    tGenLimits_ = tGenLimits;

    totalDuration_ = 0;
    rotationDuration1_ = 0;
    rotationDuration2_ = 0;
    accelDuration_ = 0;
    cruiseDuration_ = 0;
    cruiseSpeed_ = cruiseSpeed;
    newTravelHeading_ = true;

    if (cruiseRatio > 0 && cruiseRatio < 1)
        cruiseRatio_ = cruiseRatio;
    else
        cruiseRatio_ = 0.2;

    stList_.clear();
    stTimes_.clear();

    unitVec_.setZero();
    deltaVec_.setZero();
    cruiseStartPos_.setZero();
    cruiseEndPos_.setZero();
    cruiseVel_.setZero();

    LongTrajectory::initTrajectory();
}

/**
 * Initialize the short range trajectories
 */
void LongTrajectory::initTrajectory()
{
    qStart_ = wStart_->quaternion().normalized();
    qEnd_ = wEnd_->quaternion().normalized();

    // Determine travel attitude
    double dx = wEnd_->posI()(0) - wStart_->posI()(0);
    double dy = wEnd_->posI()(1) - wStart_->posI()(1);
    double dz = wEnd_->posI()(2) - wStart_->posI()(2);
    double xyDistance = sqrt(dx * dx + dy * dy);
    double travelHeading = 0;

    if ((xyDistance != 0) && (atan(fabs(dz) / xyDistance) < tGenLimits_->maxPathInclination()))
    { // Trajectory pitch is ok
        travelHeading = atan2(dy, dx); // Radians
        qCruise_ = auv_core::math_lib::toQuaternion(travelHeading, 0.0, 0.0); // yaw, pitch, roll --> quaternion
    }
    else
    {                              // Trajectory pitch is too steep, do not set a new travel heading
        newTravelHeading_ = false; // Motion is primarily in z-direction (no need to turn to heading)
        qCruise_ = qStart_;
    }

    LongTrajectory::initWaypoints();
    LongTrajectory::initSimultaneousTrajectories();
}

void LongTrajectory::initWaypoints()
{
    // Init travel vectors
    deltaVec_ = wEnd_->posI() - wStart_->posI();
    unitVec_ = deltaVec_.normalized();
    double accelDistance = deltaVec_.norm() * (1.0 - cruiseRatio_) / 2.0;

    // Calculate accel duration
    Eigen::Vector4d transStart = Eigen::Vector4d::Zero();
    Eigen::Vector4d transEnd = Eigen::Vector4d::Zero();
    transStart << 0, 0, 0, tGenLimits_->xyzJerk(accelDistance);
    transEnd << accelDistance, 0, 0, tGenLimits_->xyzJerk(accelDistance);
    MinJerkTimeSolver *mjts;
    mjts = new MinJerkTimeSolver(transStart, transEnd);
    accelDuration_ = mjts->getTime();

    // Init position vectors and cruise duration
    cruiseStartPos_ = wStart_->posI() + accelDistance * unitVec_;
    cruiseEndPos_ = wEnd_->posI() - accelDistance * unitVec_;
    cruiseVel_ = cruiseSpeed_ * unitVec_;
    cruiseDuration_ = deltaVec_.norm() * cruiseRatio_ / cruiseSpeed_;

    // Init waypoints: pre-translate -> cruise start -> cruise end -> post-translate
    // Pre/Post translate waypoints are at rest
    Eigen::Vector3d zero3d = Eigen::Vector3d::Zero();
    wPreTranslate_ = new Waypoint(wStart_->posI(), zero3d, zero3d, qCruise_, zero3d);
    wCruiseStart_ = new Waypoint(cruiseStartPos_, cruiseVel_, zero3d, qCruise_, zero3d);
    wCruiseEnd_ = new Waypoint(cruiseEndPos_, cruiseVel_, zero3d, qCruise_, zero3d);
    wPostTranslate_ = new Waypoint(wEnd_->posI(), zero3d, zero3d, qCruise_, zero3d);
}

void LongTrajectory::initSimultaneousTrajectories()
{
    totalDuration_ = 0;
    stList_.clear();
    stTimes_.clear();
    Eigen::Quaterniond qDiff = Eigen::Quaterniond::Identity();

    if (newTravelHeading_)
    {
        qDiff = qStart_.conjugate() * qCruise_;
        rotationDuration1_ = LongTrajectory::computeRotationTime(qDiff);
        stPreRotation_ = new SimultaneousTrajectory(wStart_, wPreTranslate_, rotationDuration1_);
        stList_.push_back(stPreRotation_);
        totalDuration_ += rotationDuration1_;
        stTimes_.push_back(totalDuration_);
    }

    stSpeedUp_ = new SimultaneousTrajectory(wPreTranslate_, wCruiseStart_, accelDuration_);
    stList_.push_back(stSpeedUp_);
    totalDuration_ += accelDuration_;
    stTimes_.push_back(totalDuration_);

    stCruise_ = new SimultaneousTrajectory(wCruiseStart_, wCruiseEnd_, cruiseDuration_);
    stList_.push_back(stCruise_);
    totalDuration_ += cruiseDuration_;
    stTimes_.push_back(totalDuration_);

    stSlowDown_ = new SimultaneousTrajectory(wCruiseEnd_, wPostTranslate_, accelDuration_);
    stList_.push_back(stSlowDown_);
    totalDuration_ += accelDuration_;
    stTimes_.push_back(totalDuration_);

    qDiff = qCruise_.conjugate() * qEnd_;
    rotationDuration2_ = LongTrajectory::computeRotationTime(qDiff);
    stPostRotation_ = new SimultaneousTrajectory(wPostTranslate_, wEnd_, rotationDuration2_);
    stList_.push_back(stPostRotation_);
    totalDuration_ += rotationDuration2_;
    stTimes_.push_back(totalDuration_);

    std::cout << "LT: set travel heading duration: " << rotationDuration1_ << std::endl;
    std::cout << "LT: speed up duration: " << accelDuration_ << std::endl;
    std::cout << "LT: cruise duration: " << cruiseDuration_ << std::endl;
    std::cout << "LT: slow down duration: " << accelDuration_ << std::endl;
    std::cout << "LT: final rotation duration: " << rotationDuration2_ << std::endl;
}

/**
 * @param qDiff Difference quaternion wrt B-frame (qDiff = q1.conjugate * q2)
 * Compute rotation duration given difference quaternion and TGenLimits
 */
double LongTrajectory::computeRotationTime(Eigen::Quaterniond qDiff)
{
    double angularDistance = auv_core::math_lib::quaternion2AngleAxis(qDiff)(0);

    Eigen::Vector4d rotStart = Eigen::Vector4d::Zero();
    Eigen::Vector4d rotEnd = Eigen::Vector4d::Zero();
    rotStart << 0, 0, 0, tGenLimits_->rotJerk(angularDistance);
    rotEnd << angularDistance, 0, 0, tGenLimits_->rotJerk(angularDistance);

    MinJerkTimeSolver *mjts;
    mjts = new MinJerkTimeSolver(rotStart, rotEnd);

    return mjts->getTime();
}

double LongTrajectory::getTime()
{
    return totalDuration_;
}

/**
 * @param time Time to compute the state at
 * Computes the trajectory state at the specified time
 */
Vector13d LongTrajectory::computeState(double time)
{
    if (time < 0)
        return stList_.front()->computeState(time);
    if (time > totalDuration_)
        return stList_.back()->computeState(time);

    for (int i = 0; i < stList_.size(); i++)
    {
        if (time < stTimes_[i])
        {
            double t = (i==0) ? time : time- stTimes_[i-1];
            //std::cout << "BT: compute state from ST " << i << "at time " << t << std::endl;
            return stList_[i]->computeState(t);
        }
    }
}

Vector6d LongTrajectory::computeAccel(double time)
{
    if (time < 0)
        return stList_.front()->computeAccel(time);
    if (time > totalDuration_)
        return stList_.back()->computeAccel(time);

    for (int i = 0; i < stList_.size(); i++)
    {
        if (time < stTimes_[i])
        {
            double t = (i==0) ? time : time - stTimes_[i-1];
            return stList_[i]->computeAccel(t);
        }
    }
}
} // namespace auv_guidance