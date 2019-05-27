#include "auv_gnc/trajectory_line.hpp"

namespace AUV_GNC
{
namespace Translation
{
/**
 * @param initialPos Initial position in inertial-frame [x; y; z] in [m]
 * @param nominalSpeed Nominal travel speed, in [m/s]
 * @param acceleration Desired (absolute) acceleration, in [m/s^2]
 * @param seq Acceleration sequence (SEQ_NONE, SEQ_START, SEQ_END, SEQ_BOTH). See DistanceMotionPlanner for actual names
 */
Line::Line(const Ref<const Vector3f> initialPos, float nominalSpeed, float acceleration, int seq)
{
    initialPos_ = initialPos;
    if (nominalSpeed <= 0) // TODO: May need to be strictly less than 0, will see later
        cruiseSpeed_ = Line::DEFAULT_SPEED;
    else
        cruiseSpeed_ = abs(nominalSpeed);
    acceleration_ = acceleration;
    accelSeq_ = seq;

    initDMP_ = false;
    finalPos_.setZero();
    insertionMap_.setZero();
}

/**
 * @param finalPos Desired final position in inertial frame [x; y; z], in [m]
 */
void Line::createSegment(const Ref<const Vector3f> finalPos)
{
    finalPos_ = finalPos;
    Vector3f delta;
    delta = finalPos_ - initialPos_;
    float distance = delta.norm();

    dmp_ = new DistanceMotionPlanner(distance, cruiseSpeed_, acceleration_, accelSeq_);
    insertionMap_ = delta.normalized();
    initDMP_ = true;
}

/**
 * @brief Get travel time for this line segment
 */
float Line::getTravelTime()
{
    if (initDMP_)
        return dmp_->getTravelTime();
    else
        return -1;
}

/**
 * @param time Desired time to compute the state vector at.
 * @brief Computes the state vector at a given time instance. Returns position and velocity in inertial-frame.
 */
Vector12f Line::computeState(float time)
{
    Vector12f state;
    state.setZero();

    if (initDMP_)
    {
        Vector2f dmpState;
        dmpState = dmp_->computeState(time);

        Vector3f inertialPos = initialPos_ + insertionMap_ * dmpState(0);
        Vector3f inertialVelocity = insertionMap_ * dmpState(1);

        state.segment<3>(0) = inertialPos;
        state.segment<3>(3) = inertialVelocity;
    }

    return state;
}
} // namespace Translation
} // namespace AUV_GNC