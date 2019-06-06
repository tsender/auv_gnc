#include "auv_guidance/waypoint.hpp"

namespace AUVGuidance
{
Waypoint::Waypoint(const Ref<const Vector3d> &position, const Quaterniond &quaternion, double translationSpeed, double translationAccel,
                    double rotationSpeed, double rotationAccel)
{
    position_ = position;
    quaternion_ = quaternion;
    translationSpeed_ = translationSpeed;
    translationAccel_ = translationAccel;
    rotationSpeed_ = rotationSpeed;
    rotationAccel_ = rotationAccel;
}

Quaterniond Waypoint::quaternion()
{
    return quaternion_;
}

Vector3d Waypoint::position()
{
    return position_;
}

double Waypoint::translationSpeed()
{
    return translationSpeed_;
}

double Waypoint::translationAccel()
{
    return translationAccel_;
}

double Waypoint::rotationSpeed()
{
    return rotationSpeed_;
}

double Waypoint::rotationAccel()
{
    return rotationAccel_;
}
} // namespace AUVGuidance