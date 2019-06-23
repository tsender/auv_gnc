#ifndef AUV_CONSTANTS
#define AUV_CONSTANTS

// Useful math tools
namespace auv_core
{
namespace constants
{
// Mathematical Constants and Properties
static constexpr double GRAVITY = 9.80665;    // [m/s^2]
static constexpr double WATER_DENSITY = 1000; // [kg/m^3]

// Indeces for each state
static const int STATE_XI = 0; // Inertial X-pos, expressed in I-frame
static const int STATE_YI = 1; // Inertial Y-pos, expressed in I-frame
static const int STATE_ZI = 2; // Inertial Z-pos, expressed in I-frame
static const int STATE_U = 3;  // Inertial X velocity , expressed in B-frame
static const int STATE_V = 4;  // Inertial Y velocity , expressed in B-frame
static const int STATE_W = 5;  // Inertial Z velocity , expressed in B-frame
static const int STATE_Q1 = 6; // Quaternion (I->B Frame) i-component
static const int STATE_Q2 = 7; // Quaternion (I->B Frame) j-component
static const int STATE_Q3 = 8; // Quaternion (I->B Frame) k-component
static const int STATE_P = 9;  // Inertial X angular velocity , expressed in B-frame
static const int STATE_Q = 10; // Inertial Y angular velocity , expressed in B-frame
static const int STATE_R = 11; // Inertial Z angular velocity , expressed in B-frame
} // namespace constants
} // namespace auv_core

#endif