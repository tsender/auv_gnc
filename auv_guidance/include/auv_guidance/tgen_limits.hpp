#ifndef TGEN_LIMITS
#define TGEN_LIMITS

namespace AUVGuidance
{
class TGenLimits
{
  private:
    double maxXYVelocity_, maxXYAccel_;
    double maxZVelocity_, maxZAccel_;
    double maxRotVelocity_, maxRotAccel_;
    double xyzJerk_, xyzClosingJerk_;
    double rotJerk_, rotClosingJerk_;
    double closingTolerance_;

  public:
    TGenLimits(double maxXYVelocity, double maxXYAccel, double maxZVelocity, double maxZAccel, 
                double maxRotVelocity, double maxRotAccel,double xyzJerk, double xyzClosingJerk,
                double rotJerk, double rotClosingJerk, double closingTolerance);
    double maxXYVel();
    double maxXYAccel();
    double maxZVel();
    double maxZAccel();
    double maxRotVel();
    double maxRotAccel();
    double xyzJerk();
    double xyzClosingJerk();
    double rotJerk();
    double rotClosingJerk();
    double closingTol();
};
} // namespace AUVGuidance

#endif