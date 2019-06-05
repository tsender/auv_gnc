#ifndef POSE
#define POSE

#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;

namespace AUVGuidance
{
class Pose
{
  private:
    Vector3d position_;
    Quaterniond quaternion_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Pose(const Ref<const Vector3d> position, const Eigen::Quaterniond quaternion);
    Vector3d getPosition();
    Quaterniond getQuaternion();
};
} // namespace AUVGuidance

#endif