#ifndef MIN_JERK_TIME_SOLVER
#define MIN_JERK_TIME_SOLVER

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "auv_guidance/monotonic_trajectory_time_solver.hpp"
#include "auv_guidance/min_jerk_trajectory.hpp"

using namespace Eigen;

namespace AUVGuidance
{
// Solve for optimal time between two points given initial/final velocity, accel, and jerk
class MinJerkTimeSolver
{
private:
  // Ceres Problem
  ceres::Problem problemMTTS_;
  ceres::Solver::Options optionsMTTS_;
  ceres::Solver::Summary summaryMTTS_;
  double minTime_;

  MinJerkTrajectory *mjt_;

public:
  MinJerkTimeSolver(const Ref<const Vector4d> &start, const Ref<const Vector4d> &end);
  double getTime();
  double getMiddleVelocity();
};
} // namespace AUVGuidance

#endif