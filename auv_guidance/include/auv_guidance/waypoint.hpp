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
    Vector3d position_;
    Quaterniond quaternion_;
    double translationSpeed_;
    double translationAccel_;
    double rotationSpeed_;
    double rotationAccel_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Waypoint(const Ref<const Vector3d> &position, const Quaterniond &quaternion, double translationSpeed, double translationAccel,
            double rotationSpeed, double rotationAccel);
    Quaterniond quaternion();
    Vector3d position();
    double translationSpeed();
    double translationAccel();
    double rotationSpeed();
    double rotationAccel();
};
} // namespace AUVGuidance

#endif