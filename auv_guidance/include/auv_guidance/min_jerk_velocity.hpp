#ifndef MIN_JERK_TRAJECTORY
#define MIN_JERK_TRAJECTORY

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "auv_guidance/min_jerk_time_solver.hpp"

using namespace Eigen;

namespace AUVGuidance
{
class MinJerkVelocity
{
private:
  double c0_, c1_, c2_, c3_, c4_, c5_; // Polynomial coefficients
  double dt, dt2;
  double x0_, v0_, a0_, xf_, vf_, af_; // Initial and final conditions
  double t0_, tf_;

  // Ceres Problem
  ceres::Problem problemMinJerkTime_;
  ceres::Solver::Options optionsMinJerkTime_;
  ceres::Solver::Summary summaryMinJerkTime_;
  double minTime_;

public:
  MinJerkVelocity(const Ref<const Vector3d> &start, const Ref<const Vector3d> &end, double duration);
  void computeCoeffs();
  Vector3d computeState(double time);
};
} // namespace AUVGuidance

#endif