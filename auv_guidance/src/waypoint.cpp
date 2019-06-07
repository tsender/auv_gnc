#include "auv_guidance/waypoint.hpp"

namespace AUVGuidance
{
Waypoint::Waypoint(const Ref<const Vector3d> &x, const Ref<const Vector3d> &y, const Ref<const Vector3d> &z,
            const Quaterniond &quaternion, const Ref<const Vector3d> &angVel)
{
    x_ = x;
    y_ = y;
    z_ = z;
    quaternion_ = quaternion;
    angVel_ = angVel;
}

Vector3d Waypoint::xVec()
{
    return x_;
}

Vector3d Waypoint::yVec()
{
    return y_;
}

Vector3d Waypoint::zVec()
{
    return z_;
}

Quaterniond Waypoint::quaternion()
{
    return quaternion_;
}

Vector3d Waypoint::angVelVec()
{
    return angVel_;
}
} // namespace AUVGuidance