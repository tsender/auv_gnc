#include "auv_guidance/long_range_trajectory.hpp"

namespace AUVGuidance
{
/**
 * @param start Starting waypoint
 * @param end Ending waypoint
 * @param rotationDuration Desired rotation duration [s]
 * @param accelDuration Desired acceleration duration [s]
 * @param cruiseRatio Indicates what fraction of the total distance is to be traveled while cruising
 * @param cruiseSpeed Vehicle speed while cruising
 */
LongRangeTrajectory::LongRangeTrajectory(Waypoint *start, Waypoint *end, double rotationDuration, double accelDuration,
                                         double cruiseRatio, double cruiseSpeed)
{
    wStart_ = start;
    wEnd_ = end;
    travelDuration_ = 0;
    rotationDuration_ = rotationDuration;
    accelDuration_ = accelDuration;
    cruiseSpeed_ = cruiseSpeed;
    newTravelHeading_ = true;

    if (cruiseRatio > 0 && cruiseRatio < 1)
        cruiseRatio_ = cruiseRatio;
    else
        cruiseRatio_ = LongRangeTrajectory::DEFAULT_CRUISE_RATIO;

    srtList_.clear();
    srtTimes_.clear();

    travelVec_.setZero();
    cruiseStartPos_.setZero();
    cruiseEndPos_.setZero();
    cruiseVel_.setZero();

    LongRangeTrajectory::initTrajectory();
}

/**
 * Initialize the short range trajectories
 */
void LongRangeTrajectory::initTrajectory()
{
    qStart_ = wStart_->quaternion().normalized();
    qEnd_ = wEnd_->quaternion().normalized();

    // Determine travel attitude
    double dx = wEnd_->xI()(0) - wStart_->xI()(0);
    double dy = wEnd_->xI()(1) - wStart_->xI()(1);
    double dz = wEnd_->xI()(2) - wStart_->xI()(2);
    double xyDistance = sqrt(dx * dx + dy * dy);
    double travelHeading = 0;

    if (atan(fabs(dz) / xyDistance) < LongRangeTrajectory::MAX_TRAJECTORY_PITCH)
    { // Trajectory pitch is ok
        travelHeading = atan2(dy, dx);
        qTravel_ = AUVMathLib::toQuaternion(travelHeading, 0, 0); // yaw, pitch, roll --> quaternion
    }
    else
    {                              // Trajectory pitch is too steep, do not set a new travel heading
        newTravelHeading_ = false; // Motion is primarily in z-direction (no need to turn to heading)
        qTravel_ = qStart_;
    }

    // Initialize travel vectors and waypoints
    LongRangeTrajectory::initTravelVectors();
    LongRangeTrajectory::initWaypoints();
    LongRangeTrajectory::initShortRangeTrajectories();
}

void LongRangeTrajectory::initTravelVectors()
{
    Vector3d startPos_ = Vector3d::Zero();
    Vector3d endPos_ = Vector3d::Zero();

    startPos_(0) = wStart_->xI()(0);
    startPos_(1) = wStart_->yI()(0);
    startPos_(2) = wStart_->zI()(0);
    endPos_(0) = wEnd_->xI()(0);
    endPos_(1) = wEnd_->yI()(0);
    endPos_(2) = wEnd_->zI()(0);

    // Unit travel vector in direction of travel
    travelVec_ = (endPos_ - startPos_).normalized();

    // Distance traveled per accel period
    double accelDistance = (endPos_ - startPos_).squaredNorm() * (1 - cruiseRatio_) / 2;

    cruiseStartPos_ = startPos_ + accelDistance * travelVec_;
    cruiseEndPos_ = endPos_ - accelDistance * travelVec_;
    cruiseVel_ = cruiseSpeed_ * travelVec_;
    cruiseDuration_ = (endPos_ - startPos_).squaredNorm() * cruiseRatio_ / cruiseSpeed_;
}

void LongRangeTrajectory::initWaypoints()
{
    Vector3d zero = Vector3d::Zero();
    Vector3d xI, yI, zI;
    xI.setZero();
    yI.setZero();
    zI.setZero();

    // Cruise Waypoints
    xI(0) = cruiseStartPos_(0);
    yI(0) = cruiseStartPos_(1);
    zI(0) = cruiseStartPos_(2);
    xI(1) = cruiseVel_(0);
    yI(1) = cruiseVel_(1);
    zI(1) = cruiseVel_(2);
    wCruiseStart_ = new Waypoint(xI, yI, zI, qTravel_, zero);

    xI(0) = cruiseEndPos_(0);
    yI(0) = cruiseEndPos_(1);
    zI(0) = cruiseEndPos_(2); // Velocity was already set
    wCruiseEnd_ = new Waypoint(xI, yI, zI, qTravel_, zero);

    xI.setZero();
    yI.setZero();
    zI.setZero();

    // Pre/Post Translate Waypoints
    // Change to qTravel, stay at starting position, zero velocity/accel
    xI(0) = wStart_->xI()(0);
    yI(0) = wStart_->yI()(0);
    zI(0) = wStart_->zI()(0);
    wPreTranslate_ = new Waypoint(xI, yI, zI, qTravel_, zero);

    // Change to end position, stay at qTravel, zero velocity/accel
    xI(0) = wEnd_->xI()(0);
    yI(0) = wEnd_->yI()(0);
    zI(0) = wEnd_->zI()(0);
    wPostTranslate_ = new Waypoint(xI, yI, zI, qTravel_, zero);
}

void LongRangeTrajectory::initShortRangeTrajectories()
{
    travelDuration_ = 0;
    srtList_.clear();
    srtTimes_.clear();

    if (newTravelHeading_)
    {
        srtPreRotation_ = new ShortRangeTrajectory(wStart_, wPreTranslate_, rotationDuration_);
        srtList_.push_back(srtPreRotation_);

        srtTimes_.push_back(rotationDuration_);
        srtTimes_.push_back(rotationDuration_ + accelDuration_); // Account for first accelDuration_
        travelDuration_ += rotationDuration_ + accelDuration_;
    }
    else
    {
        // Add srtSpeedUp_ info
        srtTimes_.push_back(accelDuration_);
        travelDuration_ += accelDuration_;
    }

    srtSpeedUp_ = new ShortRangeTrajectory(wPreTranslate_, wCruiseStart_, accelDuration_);
    srtList_.push_back(srtSpeedUp_);

    srtCruise_ = new ShortRangeTrajectory(wCruiseStart_, wCruiseEnd_, cruiseDuration_);
    srtList_.push_back(srtCruise_);
    srtTimes_.push_back(travelDuration_ + cruiseDuration_);
    travelDuration_ += cruiseDuration_;

    srtSlowDown_ = new ShortRangeTrajectory(wCruiseEnd_, wPostTranslate_, accelDuration_);
    srtList_.push_back(srtSlowDown_);
    srtTimes_.push_back(travelDuration_ + accelDuration_);
    travelDuration_ += accelDuration_;

    srtPostRotation_ = new ShortRangeTrajectory(wPostTranslate_, wEnd_, rotationDuration_);
    srtList_.push_back(srtPostRotation_);
    srtTimes_.push_back(travelDuration_ + rotationDuration_);
    travelDuration_ += rotationDuration_;
}

/**
 * @param time Time to compute the state at
 * Computes the trajectory state at the specified time
 */
Vector12d LongRangeTrajectory::computeState(double time)
{
    if (time < 0)
        return srtList_.front()->computeState(time);
    if (time > travelDuration_)
        return srtList_.back()->computeState(time);

    for (int i = 0; i < srtList_.size(); i++)
        if (time < srtTimes_[i])
            return srtList_[i]->computeState(srtTimes_[i] - time);
}

Vector6d LongRangeTrajectory::computeAccel(double time)
{
    if (time < 0)
        return srtList_.front()->computeAccel(time);
    if (time > travelDuration_)
        return srtList_.back()->computeAccel(time);

    for (int i = 0; i < srtList_.size(); i++)
        if (time < srtTimes_[i])
            return srtList_[i]->computeAccel(srtTimes_[i] - time);
}
} // namespace AUVGuidance