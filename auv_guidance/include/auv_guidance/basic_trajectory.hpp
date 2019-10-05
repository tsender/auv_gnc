#ifndef BASIC_TRAJECTORY
#define BASIC_TRAJECTORY

#include "auv_core/auv_core_headers.hpp"
#include "auv_guidance/abstract_trajectory.hpp"
#include "auv_guidance/waypoint.hpp"
#include "auv_guidance/min_jerk_time_solver.hpp"
#include "auv_guidance/min_jerk_trajectory.hpp"
#include "auv_guidance/simultaneous_trajectory.hpp"
#include "auv_guidance/long_trajectory.hpp"
#include "auv_guidance/tgen_limits.hpp"

#include "math.h"
#include <algorithm>

namespace auv_guidance
{
class BasicTrajectory : public Trajectory
{
private:
   TGenLimits *tGenLimits_;
   SimultaneousTrajectory *stStop_, *stPrimary_;
   LongTrajectory *ltPrimary_;
   MinJerkTrajectory *mjtHelper_;
   Waypoint *wStart_, *wStop_, *wEnd_;
   Eigen::Quaterniond qStop_, qEnd_;

   Eigen::Vector3d unitVec_, deltaVec_, maxVelocityVec_;
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
   auv_core::Vector13d computeState(double time);
   auv_core::Vector6d computeAccel(double time);
};
} // namespace auv_guidance

#endif