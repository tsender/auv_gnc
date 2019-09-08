#ifndef POSE_EDKF_INTERFACE
#define POSE_EDKF_INTERFACE

#include "auv_core/rot3d.hpp"
#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include "math.h"
#include <sstream>
#include <cppad/cppad.hpp>
#include <tf/tf.h>
#include "geometry_msgs/Vector3.h"
#include <eigen_conversions/eigen_msg.h>

using namespace std;
//using namespace CppAD;
using CppAD::AD;
typedef Eigen::Matrix<AD<double>, Eigen::Dynamic, Eigen::Dynamic> ADMatrixXd;
typedef Eigen::Matrix<AD<double>, Eigen::Dynamic, 1> ADVectorXd;

namespace auv_navigation
{
class TestNode
{
private:
  ros::NodeHandle nh;
  Eigen::Matrix3d mat;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TestNode();
  void copy(const Eigen::Ref<const Eigen::MatrixXd> &m);
  Eigen::Vector4d multiplyQuaternions(Eigen::Vector4d q1, Eigen::Vector4d q2);
  Eigen::Matrix4d quaternionMatrix(Eigen::Vector4d q);
};
} // namespace auv_navigation

#endif