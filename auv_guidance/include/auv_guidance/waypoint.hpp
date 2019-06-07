#ifndef WAYPOINT
#define WAYPOINT

#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;

namespace AUVGuidance
{
class Waypoint
{
  private:
    Vector3d x_, y_, z_; // Position, velocity, and acceleration for each axis
    Quaterniond quaternion_; // Attitude
    Vector3d angVel_; // Angular velocity

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Waypoint(const Ref<const Vector3d> &x, const Ref<const Vector3d> &y, const Ref<const Vector3d> &z,
            const Quaterniond &quaternion, const Ref<const Vector3d> &angVel);
    Vector3d xVec();
    Vector3d yVec();
    Vector3d zVec();
    Quaterniond quaternion();
    Vector3d angVelVec();
};
} // namespace AUVGuidance

#endif