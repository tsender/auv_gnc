#include "auv_guidance/pose.hpp"

namespace AUVGuidance
{
PoseL::Pose(const Ref<const Vector3d> position, const Eigen::Quaterniond quaternion)
{
    position_ = position;
    quaternion_ = quaternion;
}

Vector3d Pose::getPosition()
{
    return position_;
}

Quaterniond Pose::getQuaternion()
{
    return quaternion_;
}
} // namespace AUVGuidance