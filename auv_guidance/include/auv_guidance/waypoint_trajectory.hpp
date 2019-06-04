#ifndef WAYPOINT_TRAJECTORY
#define WAYPOINT_TRAJECTORY

#include "auv_guidance/trajectory_abstract.hpp"
#include "auv_guidance/segment_planner.hpp"
#include "auv_navigation/auv_math_lib.hpp"
#include "eigen3/Eigen/Dense"
#include "math.h"
#include <vector>

using namespace Eigen;

namespace AUVGuidance
{
typedef Matrix<double, 7, 1> Vector7d;
// Crea the trajectory that moves the vehicle from one pose thru a series of waypoints to the final pose.
class WaypointTrajectory : public Trajectory
{
private:
  SegmentPlanner *mainSegPlanner_, *arcSegPlanner_;
  double cruiseSpeed_, acceleration_;
  int accelSeq_;
  Vector7d startPose_, endPose_; // X, Y, Z, q0, q1, q2, q3 (position and attitude only)
  std::vector<Matrix3d> arcVectors_;
  bool moreThan2_; // True if more than two points in trajectory
  // TODO: add max depth velocity feature

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const float DEFAULT_SPEED = 0.5; // [m/s]
  static const float DEFAULT_ACCEL = 0.2; // [m/s^2]

  WaypointTrajectory(const Ref<const Vector7d> startPose, const Ref<const Vector7d> endPose, bool moreThan2,
                     double transSpeed, double transAccel, double rotSpeed, double rotAccel);
  float getTravelTime();
  void mapTrajectory();
  Vector12d computeState(double time);
};
} // namespace AUVGuidance

#endif