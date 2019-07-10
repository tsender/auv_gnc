#include "auv_guidance/tgen_limits.hpp"

namespace auv_guidance
{
TGenLimits::TGenLimits(double maxXYDistance, double maxZDistance, double maxXYVelocity, double maxXYAccel,
                       double maxZVelocity, double maxZAccel, double maxRotVelocity, double maxRotAccel,
                       double xyzJerk, double xyzClosingJerk, double rotJerk, double rotClosingJerk,
                       double closingTolerance, double maxPathInclination)
{
    maxXYDistance_ = maxXYDistance;
    maxZDistance_ = maxZDistance;
    maxXYVelocity_ = fabs(maxXYVelocity);
    maxXYAccel_ = fabs(maxXYAccel);
    maxZVelocity_ = fabs(maxZVelocity);
    maxZAccel_ = fabs(maxZAccel);
    maxRotVelocity_ = fabs(maxRotVelocity);
    maxRotAccel_ = fabs(maxRotAccel);

    xyzJerk_ = fabs(xyzJerk);
    xyzClosingJerk_ = fabs(xyzClosingJerk);
    rotJerk_ = fabs(rotJerk);
    rotClosingJerk_ = fabs(rotClosingJerk);
    closingTolerance_ = fabs(closingTolerance);

    maxPathInclination_ = fabs(maxPathInclination);
    if (maxPathInclination_ > 90.0)
    {
        maxPathInclination_ = 90.0;
    }
    maxPathInclination_ *= M_PI / 180.0; // Convert to radians
}

double TGenLimits::maxXYDistance()
{
    return maxXYDistance_;
}

double TGenLimits::maxZDistance()
{
    return maxZDistance_;
}

double TGenLimits::maxXYVel()
{
    return maxXYVelocity_;
}

double TGenLimits::maxXYAccel()
{
    return maxXYAccel_;
}

double TGenLimits::maxZVel()
{
    return maxZVelocity_;
}

double TGenLimits::maxZAccel()
{
    return maxZAccel_;
}

double TGenLimits::maxRotVel()
{
    return maxRotVelocity_;
}

double TGenLimits::maxRotAccel()
{
    return maxRotAccel_;
}

double TGenLimits::xyzJerk(double distance)
{
    return (distance < closingTolerance_) ? xyzJerk_ : xyzClosingJerk_;
}

double TGenLimits::rotJerk(double distance)
{
    return (distance < closingTolerance_) ? rotJerk_ : rotClosingJerk_;
}

double TGenLimits::closingTol()
{
    return closingTolerance_;
}

double TGenLimits::maxPathInclination()
{
    return maxPathInclination_;
}
} // namespace auv_guidance