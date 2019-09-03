#include "auv_guidance/tgen_limits.hpp"

namespace auv_guidance
{
TGenLimits::TGenLimits(double maxXYDistance, double maxZDistance, double maxPathInclination, double closingTolXYZ,
                       double closingTolRot, double maxXVel, double maxYVel, double maxZVel, double maxRotVel,
                       double maxXAccel, double maxYAccel, double maxZAccel, double maxRotAccel, double xyzJerk,
                       double xyzClosingJerk, double rotJerk, double rotClosingJerk)
{
   maxXYDistance_ = fabs(maxXYDistance);
   maxZDistance_ = fabs(maxZDistance);
   closingTolXYZ_ = fabs(closingTolXYZ);
   closingTolRot_ = fabs(closingTolRot);

   maxXVel_ = fabs(maxXVel);
   maxYVel_ = fabs(maxYVel);
   maxZVel_ = fabs(maxZVel);
   maxRotVel_ = fabs(maxRotVel);

   maxXAccel_ = fabs(maxXAccel);
   maxYAccel_ = fabs(maxYAccel);
   maxZAccel_ = fabs(maxZAccel);
   maxRotAccel_ = fabs(maxRotAccel);

   xyzJerk_ = fabs(xyzJerk);
   xyzClosingJerk_ = fabs(xyzClosingJerk);
   rotJerk_ = fabs(rotJerk);
   rotClosingJerk_ = fabs(rotClosingJerk);

   maxPathInclination_ = fabs(maxPathInclination);
   if (maxPathInclination_ > M_PI / 2.0)
   {
      maxPathInclination_ = M_PI / 2.0;
   }
}

double TGenLimits::maxXYDistance()
{
   return maxXYDistance_;
}

double TGenLimits::maxZDistance()
{
   return maxZDistance_;
}

double TGenLimits::closingTolXYZ()
{
   return closingTolXYZ_;
}

double TGenLimits::closingTolRot()
{
   return closingTolRot_;
}

double TGenLimits::maxPathInclination()
{
   return maxPathInclination_;
}

double TGenLimits::maxXVel()
{
   return maxXVel_;
}

double TGenLimits::maxYVel()
{
   return maxYVel_;
}

double TGenLimits::maxZVel()
{
   return maxZVel_;
}

double TGenLimits::maxRotVel()
{
   return maxRotVel_;
}

double TGenLimits::maxXAccel()
{
   return maxXAccel_;
}

double TGenLimits::maxYAccel()
{
   return maxYAccel_;
}

double TGenLimits::maxZAccel()
{
   return maxZAccel_;
}

double TGenLimits::maxRotAccel()
{
   return maxRotAccel_;
}

double TGenLimits::xyzJerk(double distance)
{
   return (distance > closingTolXYZ_) ? xyzJerk_ : xyzClosingJerk_;
}

double TGenLimits::rotJerk(double distance)
{
   return (distance > closingTolRot_) ? rotJerk_ : rotClosingJerk_;
}

} // namespace auv_guidance