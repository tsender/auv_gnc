#ifndef BASIC_TRAJECTORY
#define BASIC_TRAJECTORY

#include "auv_guidance/abstract_trajectory.hpp"
#include "auv_guidance/waypoint.hpp"
#include "auv_guidance/minimum_jerk_trajectory.hpp"
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
  MinimumJerkTrajectory *mjtX_, *mjtY_, *mjtZ_, *mjtAtt_;
  Waypoint *start_, *end_;
  double travelTime_, translationTime_, rotationTime_;
  Quaterniond qDiff_;
  bool initialized_;
  bool simultaneous_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BasicTrajectory(Waypoint *start, Waypoint *end, double time1, double time2=0.0);
  void initTrajectory();
  Vector12d computeState(double deltaTime);
};
} // namespace AUVGuidance

#endif