#include "auv_guidance/waypoint.hpp"

namespace AUVGuidance
{
Waypoint::Waypoint(const Ref<const Vector3d> &posI, const Ref<const Vector3d> &velI, const Ref<const Vector3d> &accelI,
            const Quaterniond &quaternion, const Ref<const Vector3d> &angVelB)
{
    posI_ = posI;
    velI_ = velI;
    accelI_ = accelI;
    quaternion_ = quaternion.normalized();
    angVelB_ = angVelB;
}

Vector3d Waypoint::xI()
{
    Vector3d xI = Vector3d::Zero();
    xI << posI_(0), velI_(0), accelI_(0);
    return xI;
}

Vector3d Waypoint::yI()
{
    Vector3d yI = Vector3d::Zero();
    yI << posI_(1), velI_(1), accelI_(1);
    return yI;
}

Vector3d Waypoint::zI()
{
    Vector3d zI = Vector3d::Zero();
    zI << posI_(2), velI_(2), accelI_(2);
    return zI;
}

Vector3d Waypoint::posI()
{
    return posI_;
}

Vector3d Waypoint::velI()
{
    return velI_;
}

Vector3d Waypoint::accelI()
{
    return accelI_;
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