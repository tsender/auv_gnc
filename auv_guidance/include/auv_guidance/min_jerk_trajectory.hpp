#ifndef MIN_JERK_TRAJECTORY
#define MIN_JERK_TRAJECTORY

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

using namespace Eigen;

namespace AUVGuidance
{
class MinJerkTrajectory
{
private:
  double c0_, c1_, c2_, c3_, c4_, c5_; // Polynomial coefficients
  double dt, dt2;
  double x0_, v0_, a0_, xf_, vf_, af_; // Initial and final conditions
  double t0_, tf_;

public:
  MinJerkTrajectory(const Ref<const Vector3d> &start, const Ref<const Vector3d> &end, double duration);
  void computeCoeffs();
  Vector3d computeState(double time);
};
} // namespace AUVGuidance

#endif