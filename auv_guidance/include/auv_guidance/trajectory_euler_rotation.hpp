#ifndef TRAJECTORY_ROTATION
#define TRAJECTORY_ROTATION

#include "auv_guidance/trajectory_generator_abstract.hpp"
#include "auv_guidance/segment_planner.hpp"
#include "auv_navigation/auv_math_lib.hpp"
#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;

namespace AUV_GNC
{
namespace Trajectory
{
// Creates an Euler rotation in space using the SegmentPlanner for position/speed along the segment.
// Assumes the speed is either zero or equal to the speed entered in the constructor
class EulerRotation : public TrajectoryGenerator
{
private:
  SegmentPlanner *segPlanner_;
  float cruiseSpeed_, acceleration_, deltaTheta_;
  int accelSeq_, eulerAngle_;
  Vector3f initialAttitude_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const float DEFAULT_SPEED = M_PI/4;  // [pi/2 rad/s] = [45 deg/s]
  static const float DEFAULT_ACCEL = M_PI; // [pi rad/s^2] = [deg/s^2]
  static const int ROLL = 0;
  static const int PITCH = 1;
  static const int YAW = 2;

  EulerRotation(const Ref<const Vector3f> initialAttitude, int eulerAngle, float deltaTheta,
                 float nominalSpeed = EulerRotation::DEFAULT_SPEED, float acceleration = 0.0, int seq = SegmentPlanner::SEQ_NONE);
  float getTravelTime();
  Vector12f computeState(float time);
};
} // namespace Trajectory
} // namespace AUV_GNC

#endif