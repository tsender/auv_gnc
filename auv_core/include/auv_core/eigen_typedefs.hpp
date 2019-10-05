#ifndef EIGEN_TYPEDEFS
#define EIGEN_TYPEDEFS

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

namespace auv_core
{
// Drag Matrix
typedef Eigen::Matrix<double, 6, 2> Matrix62d;

// Thruster matrices
typedef Eigen::Matrix<double, 5, 8> Matrix58d;
typedef Eigen::Matrix<double, 6, 8> Matrix68d;

// State Space Control Matrices
typedef Eigen::Matrix<double, 18, 18> Matrix18d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 8, 8> Matrix8d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 18, 8> Matrix18x8d;
typedef Eigen::Matrix<double, 12, 8> Matrix12x8d;
typedef Eigen::Matrix<double, 8, 18> Matrix8x18d;
typedef Eigen::Matrix<double, 8, 12> Matrix8x12d;

// Vectors
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 13, 1> Vector13d;
typedef Eigen::Matrix<double, 18, 1> Vector18d;

} // namespace auv_core

#endif