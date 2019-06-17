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
    Vector3d posI_, velI_, accelI_; // Inertial Position, velocity, and acceleration expressed in I-frame
    Quaterniond quaternion_; // Attitude wrt I-frame
    Vector3d angVelB_; // Angular velocity about B-frame axis

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Waypoint(const Ref<const Vector3d> &posI, const Ref<const Vector3d> &velI, const Ref<const Vector3d> &accelI,
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