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
    double rotationSpeed_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Waypoint(const Ref<const Vector3d> &position, const Quaterniond &quaternion, double translationSpeed, double rotationSpeed);
    Quaterniond getQuaternion();
    Vector3d getPosition();
    double getTranslationSpeed();
    double getRotationSpeed();
};
} // namespace AUVGuidance

#endif