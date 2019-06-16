#include "auv_guidance/reset_trajectory.hpp"

namespace AUVGuidance
{
/**
 * @param start Starting waypoint
 * @param end Ending waypoint
 * @param travelDuration Desired travel duration [s]
 */
ResetTrajectory::ResetTrajectory(Waypoint *start, Waypoint *end, TGenLimits *tgLimits)
{
    wStart_ = start;
    wEnd_ = end;
    tgLimits_ = tgLimits;


}

void ResetTrajectory::initTrajectory()
{
    
}

/**
 * @param time Time to compute the state at
 * Computes the trajectory state at the specified time
 */
Vector12d ResetTrajectory::computeState(double time)
{
    Vector12d state;
    state.setZero();

    
    return state;
}

/**
 * @param time Time to compute accelerations at
 * Compute inertial translational acceleration and time-derivative of angular veocity, 
 * both expressed in B-frame, at specified time
 */
Vector6d ResetTrajectory::computeAccel(double time)
{
    Vector6d accel;
    accel.setZero();

    return accel;
}
} // namespace AUVGuidance