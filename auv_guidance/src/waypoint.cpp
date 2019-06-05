#include "auv_guidance/waypoint.hpp"

namespace AUVGuidance
{
Waypoint::Waypoint(const Ref<const Vector3d> &position, const Quaterniond &quaternion, double translationSpeed, double rotationSpeed)
{
    position_ = position;
    quaternion_ = quaternion;
    translationSpeed_ = translationSpeed;
    rotationSpeed_ = rotationSpeed;
}

Quaterniond Waypoint::getQuaternion()
{
    return quaternion_;
}

Vector3d Waypoint::getPosition()
{
    return position_;
}

double Waypoint::getTranslationSpeed()
{
    return translationSpeed_;
}

double Waypoint::getRotationSpeed()
{
    return rotationSpeed_;
}
} // namespace AUVGuidance