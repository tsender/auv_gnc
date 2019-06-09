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
    start_ = start;
    end_ = end;
    rotationDuration_ = rotationDuration;
    accelDuration_ = accelDuration;
    cruiseSpeed_ = cruiseSpeed;
    newTravelHeading_ = true;

    if (cruiseRatio > 0 && cruiseRatio < 1)
        cruiseRatio_ = cruiseRatio;
    else
        cruiseRatio_ = LongRangeTrajectory::DEFAULT_CRUISE_RATIO;

    travelVec_.setZero();
    preCruisePos_.setZero();
    postCruisePos_.setZero();
    cruiseVel_.setZero();

    LongRangeTrajectory::initTrajectory();
}

/**
 * Initialize the short range trajectories
 */
void LongRangeTrajectory::initTrajectory()
{
    qStart_ = start_->quaternion().normalized();
    qEnd_ = end_->quaternion().normalized();

    // Determine travel attitude
    double dx = end_->xI()(0) - start_->xI()(0);
    double dy = end_->xI()(1) - start_->xI()(1);
    double dz = end_->xI()(2) - start_->xI()(2);
    double xyDistance = sqrt(dx * dx + dy * dy);
    double travelHeading = 0;

    if (atan(xyDistance / fabs(dz)) < LongRangeTrajectory::DEFAULT_MIN_Z_ANGLE)
    {
        newTravelHeading_ = false; // Motion is primarily in z-direction (no need to turn to heading)
        qTravel_ = qStart_;
    }
    else
    {
        travelHeading = atan2(dy, dx);
        qTravel_ = AUVMathLib::toQuaternion(travelHeading, 0, 0);
    }

    // Initialize travel vectors and waypoints
    LongRangeTrajectory::initTravelVectors();
    LongRangeTrajectory::initCruiseWaypoints();

    Vector3d zero = Vector3d::Zero();
    Vector3d xI, yI, zI;
    xI.setZero();
    yI.setZero();
    zI.setZero();

    // Change to qTravel, stay at starting position
    xI(0) = start_->xI()(0);
    yI(0) = start_->yI()(0);
    zI(0) = start_->zI()(0);
    preCruise_ = new Waypoint(xI, yI, zI, qTravel_, zero);

    // Change to end position, stay at qTravel
    xI(0) = end_->xI()(0);
    yI(0) = end_->yI()(0);
    zI(0) = end_->zI()(0);
    postCruise_ = new Waypoint(xI, yI, zI, qTravel_, zero);

    // Initialize short range trajectories
    if (newTravelHeading_)
        srtPreCruiseRot_ = new ShortRangeTrajectory(start_, preCruise_, rotationDuration_);
    srtPreCruiseTrans_ = new ShortRangeTrajectory(preCruise_, cruiseStart_, accelDuration_);
    srtPostCruiseTrans_ = new ShortRangeTrajectory(cruiseEnd_, postCruise_, accelDuration_);
    srtPostCruiseRot_ = new ShortRangeTrajectory(postCruise_, end_, rotationDuration_);
}

void LongRangeTrajectory::initTravelVectors()
{
    Vector3d startPos_ = Vector3d::Zero();
    Vector3d endPos_ = Vector3d::Zero();

    startPos_(0) = start_->xI()(0);
    startPos_(1) = start_->yI()(0);
    startPos_(2) = start_->zI()(0);
    endPos_(0) = end_->xI()(0);
    endPos_(1) = end_->yI()(0);
    endPos_(2) = end_->zI()(0);

    // Travel vector (unit vector) in direction of travel
    travelVec_ = (endPos_ - startPos_).normalized();

    // Distance traveled per short range trajectory
    double srtDistance = (endPos_ - startPos_).squaredNorm() * (1 - cruiseRatio_) / 2;

    preCruisePos_ = startPos_ + srtDistance * travelVec_;
    postCruisePos_ = endPos_ - srtDistance * travelVec_;
    cruiseVel_ = cruiseSpeed_ * travelVec_;
    cruiseDuration_ = (endPos_ - startPos_).squaredNorm() * cruiseRatio_ / cruiseSpeed_;
}

void LongRangeTrajectory::initCruiseWaypoints()
{
    Vector3d zero = Vector3d::Zero();
    Vector3d xI, yI, zI;
    xI.setZero();
    yI.setZero();
    zI.setZero();

    xI(0) = preCruisePos_(0);
    yI(0) = preCruisePos_(1);
    zI(0) = preCruisePos_(2);
    xI(1) = cruiseVel_(0);
    yI(1) = cruiseVel_(1);
    zI(1) = cruiseVel_(2);
    cruiseStart_ = new Waypoint(xI, yI, zI, qTravel_, zero);

    xI(0) = postCruisePos_(0);
    yI(0) = postCruisePos_(1);
    zI(0) = postCruisePos_(2);
    cruiseEnd_ = new Waypoint(xI, yI, zI, qTravel_, zero);
}

/**
 * @param time Time to compute the state at
 * Computes the trajectory state at the specified time
 */
Vector12d LongRangeTrajectory::computeState(double time)
{
    if (time < 0)
    {

    }
    if (time > travelDuration_)
    {

    }
    
    if (newTravelHeading_)
    {

    }
    else
    {

    }
}

Vector6d LongRangeTrajectory::computeAccel(double time)
{
    if (newTravelHeading_)
    {

    }
    else
    {
        
    }
}
} // namespace AUVGuidance