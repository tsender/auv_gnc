#ifndef RESET_TRAJECTORY
#define RESET_TRAJECTORY

#include "auv_guidance/abstract_trajectory.hpp"
#include "auv_guidance/waypoint.hpp"
#include "auv_guidance/tgen_limits"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"

using namespace Eigen;

namespace AUVGuidance
{
class ResetTrajectory : public Trajectory
{
private:
  TGenLimits *tgLimits_;
  MonotonicTrajectory *mtXYZ_, *mtAtt_;
  Waypoint *wStart_, *wEnd_;
  Quaterniond qStart_, qEnd_, qDiff_, qSlerp_;
  double xyzDistance, angularDistance_;

  Vector3d xState_, yState_, zState_, angleState_;
  Vector3d travelVec_, rotationAxis_; // Axis for rotation wrt B-frame

  bool init_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ResetTrajectory(Waypoint *start, Waypoint *end, TGenLimits *tgLimits);
  void initTrajectory();
  Vector12d computeState(double time);
  Vector6d computeAccel(double time);
};
} // namespace AUVGuidance

#endif