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

Vector3d Waypoint::posI()
{
    Vector3d pos = Vector3d::Zero();
    pos << xI_(0), yI_(0), zI_(0);
    return pos;
}

Vector3d Waypoint::velI()
{
    Vector3d vel = Vector3d::Zero();
    vel << xI_(1), yI_(1), zI_(1);
    return vel;
}

Vector3d Waypoint::accelI()
{
    Vector3d accel = Vector3d::Zero();
    accel << xI_(2), yI_(2), zI_(2);
    return accel;
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