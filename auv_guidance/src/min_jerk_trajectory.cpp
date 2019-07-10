#include "auv_guidance/min_jerk_trajectory.hpp"
#include <iostream>
namespace auv_guidance
{
/**
 * @param start Initial conditions of position, velocity, and acceleration.
 * @param end Final conditions of position, velocity, and acceleration
 * @param duration Duration for which trajectory will occur
 */
MinJerkTrajectory::MinJerkTrajectory(const Eigen::Ref<const Eigen::Vector3d> &start, const Eigen::Ref<const Eigen::Vector3d> &end, double duration)
{
    x0_ = start(0), v0_ = start(1), a0_ = start(2);
    xf_ = end(0), vf_ = end(1), af_ = end(2);
    t0_ = 0;
    tf_ = duration;
    dt = tf_ - t0_;
    dt2 = dt * dt;
    c0_ = 0, c1_ = 0, c2_ = 0, c3_ = 0, c4_ = 0, c5_ = 0;
    MinJerkTrajectory::computeCoeffs();
}

/**
 * Compute the needed coefficients for the min jerk trajectory
 */
void MinJerkTrajectory::computeCoeffs()
{
    c0_ = x0_;
    c1_ = v0_ * dt;
    c2_ = 0.5 * a0_ * dt2;
    c3_ = -10.0 * x0_ - 6.0 * v0_ * dt - 1.5 * a0_ * dt2 + 10.0 * xf_ - 4.0 * vf_ * dt + 0.5 * af_ * dt2;
    c4_ = 15.0 * x0_ + 8.0 * v0_ * dt + 1.5 * a0_ * dt2 - 15.0 * xf_ + 7.0 * vf_ * dt - af_ * dt2;
    c5_ = -6.0 * x0_ - 3.0 * v0_ * dt - 0.5 * a0_ * dt2 + 6.0 * xf_ - 3.0 * vf_ * dt + 0.5 * af_ * dt2;
}

/**
 * @param time Time instance for which to compute the state of the trajectory
 * Compute the state of the trajectory at specified time
 */
Eigen::Vector3d MinJerkTrajectory::computeState(double time)
{
    Eigen::Vector3d state = Eigen::Vector3d::Zero();
    
    if (time <= t0_)
    {
        state(0) = x0_; // Pos
        state(1) = v0_; // Vel
        state(2) = a0_; // Accel
        return state;
    }
    else if (time >= tf_)
    {
        state(0) = xf_; // Pos
        state(1) = vf_; // Vel
        state(2) = af_; // Accel
        return state;
    }
    
    double tau = (time - t0_) / (tf_ - t0_);
    double tau2 = tau * tau;
    double tau3 = tau * tau2;
    double tau4 = tau * tau3;
    double tau5 = tau * tau4;

    state(0) = c0_ + c1_ * tau + c2_ * tau2 + c3_ * tau3 + c4_ * tau4 + c5_ * tau5;
    state(1) = (c1_ + 2.0 * c2_ * tau + 3.0 * c3_ * tau2 + 4.0 * c4_ * tau3 + 5.0 * c5_ * tau4) / dt;
    state(2) = (2.0 * c2_ + 6.0 * c3_ * tau + 12.0 * c4_ * tau2 + 20.0 * c5_ * tau3) / dt2;
    return state;
}

double MinJerkTrajectory::getMiddleVelocity()
{
    Eigen::Vector3d state =  MinJerkTrajectory::computeState((tf_ - t0_) / 2.0);
    return state(1);
}

} // namespace auv_guidance