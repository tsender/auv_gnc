#ifndef BASIC_TRAJECTORY
#define BASIC_TRAJECTORY

#include "auv_guidance/abstract_trajectory.hpp"
#include "auv_guidance/waypoint.hpp"
#include "auv_guidance/min_jerk_time_solver.hpp"
#include "auv_guidance/simultaneous_trajectory.hpp"
#include "auv_guidance/tgen_limits.hpp"
#include "auv_control/auv_model.hpp"
#include "auv_navigation/auv_math_lib.hpp"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"
#include <algorithm>

using namespace Eigen;

namespace AUVGuidance
{
class BasicTrajectory : public Trajectory
{
private:
  TGenLimits *tGenLimits_;
  SimultaneousTrajectory *stRest_;
  //MonotonicTrajectory *mtXY_, *mtZ_, *mtAtt_;
  Waypoint *wStart_, *wRest_, *wEnd_;
  Quaterniond qStart_, qRest_, qEnd_, qDiff_, qSlerp_;
  double travelDuration_, angularDistance_;
  double timeRest_;

  Vector3d xState_, yState_, zState_, angleState_;
  Vector3d rotationAxis_; // Axis for rotation wrt B-frame

  bool init_, longTrajectory_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BasicTrajectory(TGenLimits *tGenLimits);
  void setWaypoints(Waypoint *wStart, Waypoint *wEnd);
  void prepSlowDown();
  void processNewWaypoint();
  Vector12d computeState(double time);
  Vector6d computeAccel(double time);
};
} // namespace AUVGuidance

#endif