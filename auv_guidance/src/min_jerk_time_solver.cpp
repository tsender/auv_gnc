#include "auv_guidance/min_jerk_time_solver.hpp"

namespace auv_guidance
{
/**
 * @param start Initial conditions of position, velocity, acceleration, and jerk
 * @param end Final conditions of position, velocity, acceleration, and jerk
 */
MinJerkTimeSolver::MinJerkTimeSolver(const Eigen::Ref<const Eigen::Vector4d> &start, const Eigen::Ref<const Eigen::Vector4d> &end)
{
    minTime_ = 0;
    problemMTTS_.AddResidualBlock(new ceres::AutoDiffCostFunction<MonotonicTrajectoryTimeSolver, 1, 1>(new MonotonicTrajectoryTimeSolver(start, end)), NULL, &minTime_);
    problemMTTS_.SetParameterLowerBound(&minTime_, 0, 0.0);
    optionsMTTS_.max_num_iterations = 100;
    optionsMTTS_.linear_solver_type = ceres::DENSE_QR;

    ceres::Solve(optionsMTTS_, &problemMTTS_, &summaryMTTS_);
}

/**
 * Returns the time calculated by the solver
 */
double MinJerkTimeSolver::getDuration()
{
    return minTime_;
}
} // namespace auv_guidance