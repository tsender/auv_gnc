#ifndef WAYPOINT_TRAJECTORY
#define WAYPOINT_TRAJECTORY

#include "auv_guidance/trajectory_abstract.hpp"
#include "auv_guidance/waypoint.hpp"
#include "auv_guidance/segment_planner.hpp"
#include "auv_navigation/auv_math_lib.hpp"
#include "eigen3/Eigen/Dense"
#include "math.h"
#include <vector>

using namespace Eigen;

namespace AUVGuidance
{
class WaypointTrajectory : public Trajectory
{
private:
  SegmentPlanner *translationSegPlanner_, *rotationSegPlanner_;
  std::vector<Waypoint*> waypoints_;
  Waypoint *lastWaypoint_;
  double travelTime_;
  bool initialized_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const float DEFAULT_SPEED = 0.5; // [m/s]
  static const float DEFAULT_ACCEL = 0.2; // [m/s^2]

  WaypointTrajectory(Waypoint *start, Waypoint *end);
  void addWaypoint(Waypoint *waypoint);
  void initNextTrajectory();
  Vector12d computeState(double deltaTime);
};
} // namespace AUVGuidance

#endif