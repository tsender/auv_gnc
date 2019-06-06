#ifndef BASIC_TRAJECTORY
#define BASIC_TRAJECTORY

#include "auv_guidance/trajectory_abstract.hpp"
#include "auv_guidance/waypoint.hpp"
#include "auv_guidance/segment_planner.hpp"
#include "auv_control/auv_model.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"
#include <vector>

using namespace Eigen;

namespace AUVGuidance
{
class BasicTrajectory : public Trajectory
{
private:
  SegmentPlanner *translationSegPlanner_, *rotationSegPlanner_;
  Waypoint *start_, *end_;
  double translationTime_, rotationTime_, maxTime_;
  bool initialized_;
  Vector3d delta_, insertionMap_;
  Quaterniond qDiff_;

  enum motions
  {
    translation = 0,
    rotation = 1,
    both = 2
  };

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BasicTrajectory(Waypoint *start, Waypoint *end);
  void initTrajectory();
  Vector12d computeState(double deltaTime);
};
} // namespace AUVGuidance

#endif