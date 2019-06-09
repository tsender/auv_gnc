#ifndef SHORT_RANGE_TRAJECTORY
#define SHORT_RANGE_TRAJECTORY

#include "auv_guidance/abstract_trajectory.hpp"
#include "auv_guidance/waypoint.hpp"
#include "auv_guidance/min_jerk_trajectory.hpp"
#include "auv_control/auv_model.hpp"
#include "auv_navigation/auv_math_lib.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"

using namespace Eigen;

namespace AUVGuidance
{
typedef Matrix<double, 6, 1> Vector6d;
class ShortRangeTrajectory : public Trajectory
{
private:
  MinJerkTrajectory *mjtX_, *mjtY_, *mjtZ_, *mjtAtt_;
  Waypoint *start_, *end_;
  Quaterniond qStart_, qEnd_, qDiff_, qSlerp_;
  double travelDuration_, angularDistance_;

  Vector3d xState_, yState_, zState_, angleState_;
  Vector3d rotationAxis_; // Axis for rotation wrt B-frame

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ShortRangeTrajectory(Waypoint *start, Waypoint *end, double travelDuration);
  void initTrajectory();
  Vector12d computeState(double time);
  Vector6d computeAccel(double time);
};
} // namespace AUVGuidance

#endif