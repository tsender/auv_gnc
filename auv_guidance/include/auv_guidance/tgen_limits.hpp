#ifndef TGEN_LIMITS
#define TGEN_LIMITS

#include <math.h>

namespace auv_guidance
{
class TGenLimits
{
private:
   // SI units - m, m/s, m/s^2, m/s^3
   double maxXYDistance_, maxZDistance_, maxPathInclination_, closingTolXYZ_, closingTolRot_;
   double maxXVel_, maxYVel_, maxZVel_, maxRotVel_;
   double maxXAccel_, maxYAccel_, maxZAccel_, maxRotAccel_;
   double xyzJerk_, xyzClosingJerk_;
   double rotJerk_, rotClosingJerk_;

public:
   TGenLimits(double maxXYDistance, double maxZDistance, double maxPathInclination, double closingTolXYZ,
              double closingTolRot, double maxXVel, double maxYVel, double maxZVel, double maxRotVel, double maxXAccel,
              double maxYAccel, double maxZAccel, double maxRotAccel, double xyzJerk, double xyzClosingJerk,
              double rotJerk, double rotClosingJerk);
   double maxXYDistance();
   double maxZDistance();
   double closingTolXYZ();
   double closingTolRot();
   double maxPathInclination();
   double maxXVel();
   double maxYVel();
   double maxZVel();
   double maxRotVel();
   double maxXAccel();
   double maxYAccel();
   double maxZAccel();
   double maxRotAccel();
   double xyzJerk(double distance);
   double rotJerk(double distance);
};
} // namespace auv_guidance

#endif