#include "auv_guidance/min_jerk_time_solver.hpp"

namespace AUVGuidance
{
/**
 * @param start Initial conditions of position, velocity, and acceleration.
 * @param end Final conditions of position, velocity, and acceleration
 * @param duration Duration for which trajectory will occur
 */
MinJerkTimeSolver::MinJerkTimeSolver(const Eigen::Ref<const Eigen::Vector4d> &start, const Eigen::Ref<const Eigen::Vector4d> &end)
{
    minTime_ = 0;
    problemMTTS_.AddResidualBlock(new ceres::AutoDiffCostFunction<MonotonicTrajectoryTimeSolver, 1, 1>(new MonotonicTrajectoryTimeSolver(start, end)), NULL, &minTime_);
    problemMTTS_.SetParameterLowerBound(&minTime_, 0, 0.0);
    optionsMTTS_.max_num_iterations = 100;
    optionsMTTS_.linear_solver_type = ceres::DENSE_QR;

    ceres::Solve(optionsMTTS_, &problemMTTS_, &summaryMTTS_);

    mjt_ = new MinJerkTrajectory(start.head<3>(), end.head<3>(), minTime_);
}

/**
 * Returns the time calculated by the solver
 */
double MinJerkTimeSolver::getTime()
{
    return minTime_;
}

double MinJerkTimeSolver::getMiddleVelocity()
{
    Eigen::Vector3d state =  mjt_->computeState(minTime_ / 2.0);
    return state(1);
}
} // namespace AUVGuidance