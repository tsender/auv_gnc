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
    Vector3d xI_, yI_, zI_; // Position, velocity, and acceleration wrt to Inertial-frame for each axis
    Quaterniond quaternion_; // Attitude wrt Inertial-frame
    Vector3d angVelB_; // Angular velocity about Body-frame axis

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Waypoint(const Ref<const Vector3d> &xI, const Ref<const Vector3d> &yI, const Ref<const Vector3d> &zI,
            const Quaterniond &quaternion, const Ref<const Vector3d> &angVelB);
    Vector3d xI();
    Vector3d yI();
    Vector3d zI();
    Vector3d posI();
    Vector3d velI();
    Vector3d accelI();
    Quaterniond quaternion();
    Vector3d angVelB();
};
} // namespace AUVGuidance

#endif