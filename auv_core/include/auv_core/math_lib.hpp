#ifndef MATH_LIB
#define MATH_LIB

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"

// Useful math tools
namespace auv_core
{
namespace math_lib
{
int sign(double x);

int sign(float x);

int sign(int x);

Eigen::MatrixXf sign(const Eigen::Ref<const Eigen::MatrixXf> &mat);

Eigen::MatrixXd sign(const Eigen::Ref<const Eigen::MatrixXd> &mat);
} // namespace math_lib
} // namespace auv_core

#endif