#ifndef TGEN_LIMITS
#define TGEN_LIMITS

#include <math.h>

namespace auv_guidance
{
class TGenLimits
{
private:
  // SI units - m, m/s, m/s^2, m/s^3
  double maxXYDistance_, maxZDistance_;
  double maxXYVelocity_, maxXYAccel_;
  double maxZVelocity_, maxZAccel_;
  double maxRotVelocity_, maxRotAccel_;
  double xyzJerk_, xyzClosingJerk_;
  double rotJerk_, rotClosingJerk_;
  double closingTolerance_;
  double maxPathInclination_; // [rad], if below this angle, will turn heading towards endpoint

public:
  TGenLimits(double maxXYDistance, double maxZDistance, double maxXYVelocity, double maxXYAccel,
             double maxZVelocity, double maxZAccel, double maxRotVelocity, double maxRotAccel,
             double xyzJerk, double xyzClosingJerk, double rotJerk, double rotClosingJerk,
             double closingTolerance, double maxPathInclination);
  double maxXYDistance();
  double maxZDistance();
  double maxXYVel();
  double maxXYAccel();
  double maxZVel();
  double maxZAccel();
  double maxRotVel();
  double maxRotAccel();
  double xyzJerk(double distance);
  double rotJerk(double distance);
  double closingTol();
  double maxPathInclination();
};
} // namespace auv_guidance

#endif