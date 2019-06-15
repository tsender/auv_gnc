#ifndef MONOTONIC_TRAJECTORY
#define MONOTONIC_TRAJECTORY

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "auv_guidance/monotonic_trajectory_base.hpp"
#include <vector>
#include "math.h"

using namespace Eigen;

namespace AUVGuidance
{
// The monotonic trajectory time solver can fail if the signs on the velocity do not follow a general
// convention. This class takes care of that problem.
class MonotonicTrajectory
{
private:
  std::vector<MonotonicTrajectoryBase*> mtbList_;
  std::vector<double> mtbTimes_;

  Vector4d start_, end_;
  double totalTime_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  MonotonicTrajectory(const Ref<const Vector4d> &start, const Ref<const Vector4d> &end, double maxAccel);
  double getTime();
  Vector3d computeState(double time);
};
} // namespace AUVGuidance

#endif