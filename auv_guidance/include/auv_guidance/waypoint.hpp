#ifndef WAYPOINT
#define WAYPOINT

#include "eigen3/Eigen/Dense"
#include "math.h"

namespace AUVGuidance
{
class Waypoint
{
  private:
    Eigen::Vector3d posI_, velI_, accelI_; // Inertial Position, velocity, and acceleration expressed in I-frame
    Eigen::Quaterniond quaternion_; // Attitude wrt I-frame
    Eigen::Vector3d angVelB_; // Angular velocity about B-frame axis

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Waypoint(const Eigen::Ref<const Eigen::Vector3d> &posI, const Eigen::Ref<const Eigen::Vector3d> &velI, const Eigen::Ref<const Eigen::Vector3d> &accelI,
            const Eigen::Quaterniond &quaternion, const Eigen::Ref<const Eigen::Vector3d> &angVelB);
    Eigen::Vector3d xI();
    Eigen::Vector3d yI();
    Eigen::Vector3d zI();
    Eigen::Vector3d posI();
    Eigen::Vector3d velI();
    Eigen::Vector3d accelI();
    Eigen::Quaterniond quaternion();
    Eigen::Vector3d angVelB();
};
} // namespace AUVGuidance

#endif