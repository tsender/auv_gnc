#include "auv_guidance/trajectory_arc.hpp"

namespace AUV_GNC
{
namespace Trajectory
{
/**
 * @param initialPos Initial position in inertial-frame [x; y; z], in [m].
 * @param unitTangent Unit vector tangent to initial velocity.
 * @param unitNormal Unit vector normal to unitTangent, extending from initialPos towards rotation axis.
 * @param radius Radius of arc, in [m].
 * @param theta Angle of revolution for the arc, in [deg]
 * @param acceleration Desired (absolute) acceleration, in [m/s^2].
 * @param seq Acceleration sequence, can be one of SegmentPlanner::(SEQ_NONE, SEQ_START, SEQ_END, SEQ_BOTH)
 */
Arc::Arc(const Ref<const Vector3f> initialPos, const Ref<const Vector3f> unitTangent, const Ref<const Vector3f> unitNormal,
         float radius, float theta, float nominalSpeed, float acceleration, int seq)
{
    initialPos_ = initialPos;
    unitTangent_ = unitTangent.normalized();
    unitNormal_ = unitNormal.normalized();
    radius_ = abs(radius);
    theta_ = abs(theta);
    insertionMap_.col(0) = unitTangent_;
    insertionMap_.col(1) = unitNormal_;

    if (nominalSpeed <= 0) // TODO: May need to be strictly less than 0, will see later
        cruiseSpeed_ = Arc::DEFAULT_SPEED;
    else
        cruiseSpeed_ = abs(nominalSpeed);
    acceleration_ = acceleration;
    accelSeq_ = seq;

    float arcLength = radius_ * theta_ * M_PI / 180.0;
    segPlanner_ = new SegmentPlanner(arcLength, cruiseSpeed_, acceleration_, accelSeq_);
}

/**
 * @brief Get travel time for this arc segment, in [s].
 */
float Arc::getTravelTime()
{
    return segPlanner_->getTravelTime();
}

/**
 * @param time Desired time to compute the state vector at.
 * @brief Computes the state vector at a given time instance. Returns position and velocity in inertial-frame.
 */
Vector12f Arc::computeState(float time)
{
    Vector12f state;
    state.setZero();

    Vector2f segState = segPlanner_->computeState(time);

    float phi = segState(0) / radius_;    // Current angle, in the plane of rotation
    float phiDot = segState(1) / radius_; // Current angle rate of change, in the plane of rotation

    Vector2f inPlanePos, inPlaneVelocity;
    inPlanePos.setZero();
    inPlaneVelocity.setZero();

    inPlanePos(0) = radius_ * cos(phi);                  // x = R*cos(phi)
    inPlanePos(1) = radius_ * sin(phi);                  // y = R*sin(phi)
    inPlaneVelocity(0) = -radius_ * phiDot * sin(phi); // xDot = -R*phiDot*sin(phi)
    inPlaneVelocity(1) = radius_ * phiDot * cos(phi);  // yDot = R*phiDot*cos(phi)

    Vector3f inertialPos = initialPos_ + insertionMap_ * inPlanePos;
    Vector3f inertialVelocity = insertionMap_ * inPlaneVelocity;

    state.head<3>() = inertialPos;
    state.segment<3>(6) = inertialVelocity; // Will get expressed in body-frame at later step

    return state;
}
} // namespace Translation
} // namespace AUV_GNC