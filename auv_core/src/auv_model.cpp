#include "auv_core/auv_model.hpp"

namespace auv_core
{
namespace auv
{
AUVModel::AUVModel(auvPhysics physics, auvLimits limits)
{
   physics_ = physics;
   limits_ = limits;
   thrustCoeffs_.setZero();
   AUVModel::setThrustCoeffs();
}

// Set the thruster coefficients. Each column corresponds to a single thruster.
// Rows 1,2,3: force contribution in the body-frame X, Y, and Z axes, respectively (range from 0-1)
// Rows 4,5,6: effective moment arms about body-frame the X, Y, and Z axes, respectively
void AUVModel::setThrustCoeffs()
{
   thrustCoeffs_.setZero();

   for (int i = 0; i < physics_.numThrusters; i++)
   {
      double psi = physics_.thrusterData(3, i) * M_PI / 180; // Yaw
      double theta = physics_.thrusterData(4, i) * M_PI / 180; // Pitch

      // Compute effective contribution of thrust along XYZ axes, value in range [0, 1]
      thrustCoeffs_(0, i) = cos(psi) * cos(theta); // cos(psi)cos(theta)
      thrustCoeffs_(1, i) = sin(psi) * cos(theta); // sin(psi)cos(theta)
      thrustCoeffs_(2, i) = -sin(theta);           // -sin(theta)

      // Compute effective moment arm for each thruster along XYZ axes
      thrustCoeffs_.block<3, 1>(3, i) = physics_.thrusterData.block<3, 1>(0, i).cross(thrustCoeffs_.block<3, 1>(0, i));
   }
}

Matrix68d AUVModel::getThrustCoeffs()
{
   return thrustCoeffs_;
}

double AUVModel::maxXYDistance()
{
   return limits_.maxXYDistance;
}

double AUVModel::maxZDistance()
{
   return limits_.maxZDistance;
}

double AUVModel::closingTolXYZ()
{
   return limits_.closingTolXYZ;
}

double AUVModel::closingTolRot()
{
   return limits_.closingTolRot;
}

double AUVModel::maxPathInclination()
{
   return limits_.maxPathInclination;
}

double AUVModel::maxXVel()
{
   return limits_.maxXVel;
}

double AUVModel::maxYVel()
{
   return limits_.maxYVel;
}

double AUVModel::maxZVel()
{
   return limits_.maxZVel;
}

double AUVModel::maxRotVel()
{
   return limits_.maxRotVel;
}

double AUVModel::maxXAccel()
{
   return limits_.maxXAccel;
}

double AUVModel::maxYAccel()
{
   return limits_.maxYAccel;
}

double AUVModel::maxZAccel()
{
   return limits_.maxZAccel;
}

double AUVModel::maxRotAccel()
{
   return limits_.maxRotAccel;
}

double AUVModel::xyzJerk(double distance)
{
   return (distance > limits_.closingTolXYZ) ? limits_.xyzJerk : limits_.xyzClosingJerk;
}

double AUVModel::rotJerk(double distance)
{
   return (distance > limits_.closingTolRot) ? limits_.rotJerk : limits_.rotClosingJerk;
}

} // namespace auv
} // namespace auv_core
