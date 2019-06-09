#include "auv_guidance/waypoint.hpp"

namespace AUVGuidance
{
Waypoint::Waypoint(const Ref<const Vector3d> &xI, const Ref<const Vector3d> &yI, const Ref<const Vector3d> &zI,
            const Quaterniond &quaternion, const Ref<const Vector3d> &angVelB)
{
    xI_ = xI;
    yI_ = yI;
    zI_ = zI;
    quaternion_ = quaternion;
    angVelB_ = angVelB;
}

Vector3d Waypoint::xI()
{
    return xI_;
}

Vector3d Waypoint::yI()
{
    return yI_;
}

Vector3d Waypoint::zI()
{
    return zI_;
}

Quaterniond Waypoint::quaternion()
{
    return quaternion_;
}

Vector3d Waypoint::angVelB()
{
    return angVelB_;
}
} // namespace AUVGuidance