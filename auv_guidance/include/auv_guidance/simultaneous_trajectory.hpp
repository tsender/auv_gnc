#ifndef SIMULTANEOUS_TRAJECTORY
#define SIMULTANEOUS_TRAJECTORY

#include "auv_core/auv_core_headers.hpp"
#include "auv_guidance/abstract_trajectory.hpp"
#include "auv_guidance/waypoint.hpp"
#include "auv_guidance/min_jerk_trajectory.hpp"

#include "math.h"
#include <iostream>

namespace auv_guidance
{
// Creates a trajectory to perform both translational and rotational motion simultaneously in a specified duration
class SimultaneousTrajectory : public Trajectory
{
private:
   MinJerkTrajectory *mjtX_, *mjtY_, *mjtZ_, *mjtAtt_;
   Waypoint *wStart_, *wEnd_;
   Eigen::Quaterniond qStart_, qEnd_, qRel_, qSlerp_;
   double totalDuration_, angularDistance_;

   Eigen::Vector3d xState_, yState_, zState_, angleState_;
   Eigen::Vector3d rotationAxis_; // Axis for rotation wrt B-frame
   bool noRotation_;

public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   SimultaneousTrajectory(Waypoint *start, Waypoint *end, double duration);
   void initTrajectory();
   double getTime();
   auv_core::Vector13d computeState(double time);
   auv_core::Vector6d computeAccel(double time);
};
} // namespace auv_guidance

#endif