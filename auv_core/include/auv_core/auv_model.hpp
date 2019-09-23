#ifndef AUV_MODEL
#define AUV_MODEL

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

namespace auv_core
{
typedef Eigen::Matrix<double, 6, 2> Matrix62d;
typedef Eigen::Matrix<double, 5, 8> Matrix58d;
typedef Eigen::Matrix<double, 6, 8> Matrix68d;

/**
 * Stores AUV model properties/data
 */
struct auvModel
{
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   const double mass;             // [kg]
   const double Fg;               // [N]
   const double Fb;               // [N]
   const Eigen::Vector3d cob;     // Center of buoyancy relative to center of mass (X [m], Y [m], Z [m])
   const Eigen::Matrix3d inertia; // 3x3 inertia matrix [kg-m^2]
   const Matrix62d dragCoeffs;    // Drag coefficients (col 0: linear coeffs, col 1: quadratic coeffs, rows 0-2: translational, rows 3-5: rotational)
   const int numThrusters;
   const Matrix58d thrusterData; // Pose of thruster relative to CoM
};

/**
 * Stores AUV limits
 */
struct auvLimits
{
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   const double maxXYDistance;      // [m]
   const double maxZDistance;       // [m]
   const double maxPathInclination; // [rad]
   const double closingTolXYZ;      // [m]
   const double closingTolRot;      // [rad]
   const double maxXVel;            // [m/s]
   const double maxYVel;            // [m/s]
   const double maxZVel;            // [m/s]
   const double maxRotVel;          // [rad/s]
   const double maxXAccel;          // [m/s^2]
   const double maxYAccel;          // [m/s^2]
   const double maxZAccel;          // [m/s^2]
   const double maxRotAccel;        // [rad/s^2]
   const double xyzJerk;            // [m/s^3]
   const double xyzClosingJerk;     // [m/s^3]
   const double rotJerk;            // [rad/s^3]
   const double rotClosingJerk;     // [rad/^3]
};
} // namespace auv_core

#endif