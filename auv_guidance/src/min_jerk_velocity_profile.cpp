#include "auv_guidance/min_jerk_velocity_profile.hpp"

namespace AUVGuidance
{
/**
 * @param start Starting state (position, velocity, accel)
 * @param end Ending state (position, velocity, accel)
 * @param accelDuration Desired acceleration duration [s]
 * @param cruiseRatio Indicates what fraction of the total distance is to be traveled while cruising
 * @param cruiseSpeed Vehicle speed while cruising
 * Works best if start and end points are at rest
 */
MinJerkVelocityProfile::MinJerkVelocityProfile(const Ref<const Vector3d> &start, const Ref<const Vector3d> &end,
                                               double accelDuration, double cruiseRatio, double cruiseSpeed);
{
    startState_ = start;
    endState_ = end;
    travelDuration_ = 0;
    accelDuration_ = accelDuration;
    cruiseSpeed_ = cruiseSpeed;

    if (cruiseRatio > 0 && cruiseRatio < 1)
        cruiseRatio_ = cruiseRatio;
    else
        cruiseRatio_ = MinJerkVelocityProfile::DEFAULT_CRUISE_RATIO;

    preCruiseState_.setZero();
    postCruiseState_.setZero();

    MinJerkVelocityProfile::initTrajectory();
}

MinJerkVelocityProfile::initTrajectory()
{
    double delta = endState_(0) - startState_(0);

}

/**
 * @param time Time to compute the state at
 * Computes the trajectory state at the specified time
 */
Vector3d MinJerkVelocityProfile::computeState(double time)
{

}
} // namespace AUVGuidance