#ifndef AUV_MODEL
#define AUV_MODEL

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"

namespace auv_core
{
namespace auv
{
typedef Eigen::Matrix<double, 6, 2> Matrix62d;
typedef Eigen::Matrix<double, 5, 8> Matrix58d;
typedef Eigen::Matrix<double, 6, 8> Matrix68d;

/**
 * Makes it easier to pass this data all at once
 */
struct auvPhysics
{
   double mass;             // [kg]
   double Fg;               // [N]
   double Fb;               // [N]
   Eigen::Vector3d cob;     // Center of buoyancy relative to center of mass (X [m], Y [m], Z [m])
   Eigen::Matrix3d inertia; // 3x3 inertia matrix [kg-m^2]
   Matrix62d drag;   // Drag coefficients (first col -> linear coeffs, second col -> quadratic coeffs)
   double numThrusters;
   Matrix58d thrusterData; // Pose of thruster relative to CoM
};

/**
 * Makes it easier to pass this data all at once
 */
struct auvLimits
{
   double maxXYDistance;      // [m]
   double maxZDistance;       // [m]
   double maxPathInclination; // [rad]
   double closingTolXYZ;      // [m]
   double closingTolRot;      // [rad]
   double maxXVel;            // [m/s]
   double maxYVel;            // [m/s]
   double maxZVel;            // [m/s]
   double maxRotVel;          // [rad/s]
   double maxXAccel;          // [m/s^2]
   double maxYAccel;          // [m/s^2]
   double maxZAccel;          // [m/s^2]
   double maxRotAccel;        // [rad/s^2]
   double xyzJerk;            // [m/s^3]
   double xyzClosingJerk;     // [m/s^3]
   double rotJerk;            // [rad/s^3]
   double rotClosingJerk;     // [rad/^3]
};

// AUV
// Contains information about an AUV's attributes: mass, volume inertia, drag, and thruster properties
// Used to compute any jacobians and state vectors required by TransEKF and LQR
class AUVModel
{
private:
   auvPhysics physics_;
   auvLimits limits_;
   Matrix68d thrustCoeffs_;

   void setThrustCoeffs();

public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   AUVModel(auvPhysics physics, auvLimits limits);

   Matrix68d getThrustCoeffs();

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
} // namespace auv
} // namespace auv_core

#endif