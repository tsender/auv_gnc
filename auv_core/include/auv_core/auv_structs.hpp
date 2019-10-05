#ifndef AUV_STRUCTS
#define AUV_STRUCTS

#include "auv_core/eigen_typedefs.hpp"

namespace auv_core
{

/**
 * AUV model parameters
 */
typedef struct
{
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   double mass;             // [kg]
   double Fg;               // [N]
   double Fb;               // [N]
   Eigen::Vector3d cob;     // Center of buoyancy relative to center of mass (X [m], Y [m], Z [m])
   Eigen::Matrix3d inertia; // 3x3 inertia matrix [kg-m^2]
   auv_core::Matrix62d dragCoeffs;    // Drag coefficients (col 0: linear coeffs, col 1: quadratic coeffs, rows 0-2: translational, rows 3-5: rotational)
   int numThrusters;
   auv_core::Matrix58d thrusterData; // Pose of thruster relative to CoM
} auvParameters;

/**
 * AUV constraints
 */
typedef struct
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
} auvConstraints;
} // namespace auv_core

#endif