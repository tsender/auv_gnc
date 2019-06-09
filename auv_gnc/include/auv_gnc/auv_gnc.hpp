#ifndef AUV_TRAJECTORY
#define TRAJECTORY

#include "auv_guidance/abstract_trajectory.hpp"
#include "auv_guidance/segment_planner.hpp"
#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;

namespace AUVGuidance
{
  typedef Matrix<double, 
// Creates a line segment in space using the SegmentPlanner for position/speed along the segment.
// Assumes the speed is either zero or equal to the speed entered in the constructor
class AUVTrajectory : public Trajectory
{
private:
  SegmentPlanner *posSegPlanner_, *attSegPlanner_;
  float cruiseSpeed_, acceleration_;
  int accelSeq_;
  Vector3f initialPos_, finalPos_, insertionMap_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const float DEFAULT_SPEED = 0.5; // [m/s]
  static const float DEFAULT_ACCEL = 0.2; // [m/s^2]

  AUVTrajectory();
  void setWaypointTrajectory()
  Vector12d computeState(double time);
};
} // namespace AUV_GNC

#endif