#ifndef SIMULTANEOUS_TRAJECTORY
#define SIMULTANEOUS_TRAJECTORY

#include "auv_guidance/abstract_trajectory.hpp"
#include "auv_guidance/waypoint.hpp"
#include "auv_guidance/min_jerk_trajectory.hpp"
#include "auv_control/auv_model.hpp"
#include "auv_navigation/auv_math_lib.hpp"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"

namespace AUVGuidance
{
// Must translate along a single (arbitrary) direction
class SimultaneousTrajectory : public Trajectory
{
private:
  MinJerkTrajectory *mjtX_, *mjtY_, *mjtZ_, *mjtAtt_;
  Waypoint *wStart_, *wEnd_;
  Eigen::Quaterniond qStart_, qEnd_, qDiff_, qSlerp_;
  double totalDuration_, angularDistance_;

  Eigen::Vector3d xState_, yState_, zState_, angleState_;
  Eigen::Vector3d rotationAxis_; // Axis for rotation wrt B-frame

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimultaneousTrajectory(Waypoint *start, Waypoint *end, double duration);
  void initTrajectory();
  double getTime();
  Vector12d computeState(double time);
  Vector6d computeAccel(double time);
};
} // namespace AUVGuidance

#endif