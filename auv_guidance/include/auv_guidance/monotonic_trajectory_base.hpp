#ifndef MONOTONIC_TRAJECTORY_BASE
#define MONOTONIC_TRAJECTORY_BASE

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "auv_guidance/monotonic_trajectory_time_solver.hpp"

using namespace Eigen;

namespace AUVGuidance
{
// Base class for the mono directional trajectory
class MonotonicTrajectoryBase
{
private:
  double c0_, c1_, c2_, c3_, c4_, c5_; // Polynomial coefficients
  double dt, dt2;
  double x0_, v0_, a0_, j0_, xf_, vf_, af_, jf_; // Initial and final conditions
  double t0_, tf_;

  // Ceres Problem
  ceres::Problem problemMTTS_;
  ceres::Solver::Options optionsMTTS_;
  ceres::Solver::Summary summaryMTTS_;
  double minTime_;

public:
  MonotonicTrajectoryBase(const Ref<const Vector4d> &start, const Ref<const Vector4d> &end);
  void computeCoeffs();
  double getTime();
  Vector3d computeState(double time);
};
} // namespace AUVGuidance

#endif