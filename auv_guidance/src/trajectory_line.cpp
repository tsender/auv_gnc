#include "auv_guidance/trajectory_line.hpp"

namespace AUV_GNC
{
namespace Trajectory
{
/**
 * @param initialPos Initial position in inertial-frame [x; y; z], in [m].
 * @param finalPos Desired final position in inertial frame [x; y; z], in [m].
 * @param nominalSpeed Nominal travel speed, in [m/s].
 * @param acceleration Desired (absolute) acceleration, in [m/s^2].
 * @param seq Acceleration sequence, can be one of SegmentPlanner::(SEQ_NONE, SEQ_START, SEQ_END, SEQ_BOTH)
 */
Line::Line(const Ref<const Vector3f> initialPos, const Ref<const Vector3f> finalPos,
           float nominalSpeed, float acceleration, int seq)
{
    initialPos_ = initialPos;
    finalPos_ = finalPos;

    if (nominalSpeed <= 0) // TODO: May need to be strictly less than 0, will see later
        cruiseSpeed_ = Line::DEFAULT_SPEED;
    else
        cruiseSpeed_ = abs(nominalSpeed);
    acceleration_ = acceleration;
    accelSeq_ = seq;

    Vector3f delta;
    delta = finalPos_ - initialPos_;
    float distance = delta.norm();
    segPlanner_ = new SegmentPlanner(distance, cruiseSpeed_, acceleration_, accelSeq_);
    insertionMap_ = delta.normalized();
}

/**
 * @brief Get travel time for this line segment, in [s].
 */
float Line::getTravelTime()
{
    return segPlanner_->getTravelTime();
}

/**
 * @param time Desired time to compute the state vector at.
 * @brief Computes the state vector at a given time instance. Returns position and velocity in inertial-frame.
 */
Vector12f Line::computeState(float time)
{
    Vector12f state;
    state.setZero();

    Vector2f segState = segPlanner_->computeState(time);

    Vector3f inertialPos = initialPos_ + insertionMap_ * segState(0);
    Vector3f inertialVelocity = insertionMap_ * segState(1);

    state.head<3>() = inertialPos;
    state.segment<3>(6) = inertialVelocity; // Will get expressed in body-frame at later step

    return state;
}
} // namespace Trajectory
} // namespace AUV_GNC