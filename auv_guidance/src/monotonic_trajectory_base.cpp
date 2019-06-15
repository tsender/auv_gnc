#include "auv_guidance/monotonic_trajectory_base.hpp"

namespace AUVGuidance
{
/**
 * @param start Initial conditions of position, velocity, and acceleration.
 * @param end Final conditions of position, velocity, and acceleration
 * @param duration Duration for which trajectory will occur
 */
MonotonicTrajectoryBase::MonotonicTrajectoryBase(const Ref<const Vector4d> &start, const Ref<const Vector4d> &end)
{
    x0_ = start(0);
    v0_ = start(1);
    a0_ = start(2);
    j0_ = start(3);
    xf_ = end(0);
    vf_ = end(1);
    af_ = end(2);
    jf_ = end(3);

    t0_ = 0;
    tf_ = 0;
    minTime_ = 0;
    dt = 0;
    dt2 = 0;

    c0_ = 0, c1_ = 0, c2_ = 0, c3_ = 0, c4_ = 0, c5_ = 0;

    problemMTTS_.AddResidualBlock(new ceres::AutoDiffCostFunction<MonotonicTrajectoryTimeSolver, 1, 1>(new MonotonicTrajectoryTimeSolver(start, end)), NULL, &minTime_);
    problemMTTS_.SetParameterLowerBound(&minTime_, 0, 0.0);
    optionsMTTS_.max_num_iterations = 100;
    optionsMTTS_.linear_solver_type = ceres::DENSE_QR;

    MonotonicTrajectoryBase::computeCoeffs();
}

/**
 * Compute the needed coefficients for the mono directional trajectory
 */
void MonotonicTrajectoryBase::computeCoeffs()
{
    minTime_ = 0;
    ceres::Solve(optionsMTTS_, &problemMTTS_, &summaryMTTS_);

    tf_ = minTime_;

    dt = tf_ - t0_;
    dt2 = dt * dt;

    c0_ = v0_;
    c1_ = a0_ * dt;
    c2_ = 0.5 * j0_ * dt2;
    c3_ = -10.0 * v0_ - 6.0 * a0_ * dt - 1.5 * j0_ * dt2 + 10.0 * vf_ - 4.0 * af_ * dt + 0.5 * jf_ * dt2;
    c4_ = 15.0 * v0_ + 8.0 * a0_ * dt + 1.5 * j0_ * dt2 - 15.0 * vf_ + 7.0 * af_ * dt - jf_ * dt2;
    c5_ = -6.0 * v0_ - 3.0 * a0_ * dt - 0.5 * j0_ * dt2 + 6.0 * vf_ - 3.0 * af_ * dt + 0.5 * jf_ * dt2;
}

/**
 * Returns the time calculated by the solver
 */
double MonotonicTrajectoryBase::getTime()
{
    return minTime_;
}

/**
 * @param time Time instance for which to compute the state of teh trajectory
 * Compute the state of the trajectory at specified time
 */
Vector3d MonotonicTrajectoryBase::computeState(double time)
{
    Vector3d state = Vector3d::Zero();

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
    double tau6 = tau * tau5;

    state(0) = x0_ + (c0_ * tau + c1_ * tau2 / 2.0 + c2_ * tau3 / 3.0 + c3_ * tau4 / 4.0 + c4_ * tau5 / 5.0 + c5_ * tau6 / 6.0) * dt;
    state(1) = c0_ + c1_ * tau + c2_ * tau2 + c3_ * tau3 + c4_ * tau4 + c5_ * tau5;
    state(2) = (c1_ + 2.0 * c2_ * tau + 3.0 * c3_ * tau2 + 4.0 * c4_ * tau3 + 5.0 * c5_ * tau4) / dt;
    //state(3) = (2.0 * c2_ + 6.0 * c3_ * tau + 12.0 * c4_ * tau2 + 20.0 * c5_ * tau3) / dt2;
}
} // namespace AUVGuidance