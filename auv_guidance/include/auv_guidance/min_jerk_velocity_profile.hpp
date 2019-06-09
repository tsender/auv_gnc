#ifndef MIN_JERK_VELOCITY_PROFILE
#define MIN_JERK_VELOCITY_PROFILE

#include "auv_guidance/min_jerk_trajectory.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"

using namespace Eigen;

namespace AUVGuidance
{
class MinJerkVelocityProfile
{
private:
  MinJerkTrajectory *mjtStart_, *mjtEnd_;
  Vector3d startState_, endState_, preCruiseState_, postCruiseState_;
  double travelDuration_, accelDuration_, cruiseRatio_, cruiseSpeed_, cruiseDuration_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const float DEFAULT_CRUISE_RATIO = 0.8; // Default is to cruise for 80% of total distance

  MinJerkVelocityProfile(const Ref<const Vector3d> &start, const Ref<const Vector3d> &end,
                         double accelDuration, double cruiseRatio, double cruiseSpeed);
  Vector3d computeState(double time);
};
} // namespace AUVGuidance

#endif