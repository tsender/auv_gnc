#ifndef PARALLEL_TRAJECTORY
#define PARALLEL_TRAJECTORY

#include "auv_guidance/trajectory_generator_abstract.hpp"
#include "auv_guidance/segment_planner.hpp"
#include "auv_guidance/trajectory_line.hpp"
#include "auv_guidance/trajectory_arc.hpp"
#include "auv_guidance/trajectory_euler_rotation.hpp"
#include "auv_navigation/auv_math_lib.hpp"
#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;

namespace AUV_GNC
{
namespace Trajectory
{
// Performs both a translation and a rotation using the SegmentPlanner for position/speed along the segment.
// Assumes the speed is either zero or equal to the speed entered in the constructor
class ParallelTrajectory : public TrajectoryGenerator
{
private:
  // Need a Translation Plan and a Rotation Plan -> create Line, Arc, EulerRotation
  SegmentPlanner *spTranslation_, *spRotation_;
  float cruiseSpeed_, acceleration_, deltaTheta_;
  int accelSeq_, eulerAngle_;
  Vector3f initialAttitude_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const float DEFAULT_SPEED = 0.5;  // [m/s]
  static const float DEFAULT_ACCEL = 0.2; // [m/s^2]

  ParallelTrajectory(const Ref<const Vector3f> initialAttitude, int eulerAngle, float deltaTheta,
                 float nominalSpeed = EulerRotation::DEFAULT_SPEED, float acceleration = 0.0, int seq = SegmentPlanner::SEQ_NONE);
  float getTravelTime();
  Vector12f computeState(float time);
};
} // namespace Trajectory
} // namespace AUV_GNC

#endif