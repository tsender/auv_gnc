#include "auv_guidance/tgen_limits.hpp"

namespace AUVGuidance
{
TGenLimits::TGenLimits(double maxXYVelocity, double maxXYAccel, double maxZVelocity, double maxZAccel,
                       double maxRotVelocity, double maxRotAccel, double xyzJerk, double xyzClosingJerk,
                       double rotJerk, double rotClosingJerk, double closingTolerance)
{
    maxXYVelocity_ = maxXYVelocity_;
    maxXYAccel_ = maxXYAccel;
    maxZVelocity_ = maxZVelocity_;
    maxZAccel_ = maxZAccel;
    maxRotVelocity_ = maxRotVelocity;
    maxRotAccel_ = maxRotAccel;

    xyzJerk_ = xyzJerk;
    xyzClosingJerk_ = xyzClosingJerk;
    rotJerk_ = rotJerk;
    rotClosingJerk_ = rotClosingJerk;
    closingTolerance_ = closingTolerance;
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

double TGenLimits::xyzJerk()
{
    return xyzJerk_;
}

double TGenLimits::xyzClosingJerk()
{
    return xyzClosingJerk_;
}

double TGenLimits::rotJerk()
{
    return rotJerk_;
}

double TGenLimits::rotClosingJerk()
{
    return rotClosingJerk_;
}

double TGenLimits::closingTol()
{
    return closingTolerance_;
}
} // namespace AUVGuidance