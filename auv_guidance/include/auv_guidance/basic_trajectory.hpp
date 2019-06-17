#ifndef BASIC_TRAJECTORY
#define BASIC_TRAJECTORY

#include "auv_guidance/abstract_trajectory.hpp"
#include "auv_guidance/waypoint.hpp"
#include "auv_guidance/min_jerk_time_solver.hpp"
#include "auv_guidance/min_jerk_trajectory.hpp"
#include "auv_guidance/simultaneous_trajectory.hpp"
#include "auv_guidance/long_trajectory.hpp"
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
  SimultaneousTrajectory *stStop_, *stPrimary_;
  LongTrajectory *ltPrimary_;
  MinJerkTrajectory *mjtHelper_;
  Waypoint *wStart_, *wStop_, *wEnd_;
  Quaterniond qStop_, qEnd_;

  Vector3d unitVec_, deltaVec_, maxVelocityVec_;
  double totalDuration_, stopDuration_, simultaneousDuration_, longDuration_;
  double distance_, initialMaxVelocity_, maxVelocity_;

  bool longTrajectory_, simultaneousTrajectory_, exceedsMaxSpeed_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BasicTrajectory(Waypoint *wStart, Waypoint *wEnd, TGenLimits *tGenLimits);
  void setStopTrajectory();
  void computeMaxVelocity();
  void computeSimultaneousTime();
  void setPrimaryTrajectory();
  double getTime();
  Vector12d computeState(double time);
  Vector6d computeAccel(double time);
};
} // namespace AUVGuidance

#endif