#ifndef POSE_EDKF_INTERFACE
#define POSE_EDKF_INTERFACE

#include "auv_navigation/auv_math_lib.hpp"
#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include "math.h"
#include <sstream>
#include <cppad/cppad.hpp>

using namespace std;
//using namespace CppAD;
using CppAD::AD;
typedef Eigen::Matrix< AD<double> , Eigen::Dynamic, Eigen::Dynamic > ADMatrixXd;
typedef Eigen::Matrix< AD<double> , Eigen::Dynamic, 1 > ADVectorXd;

namespace AUVNavigation
{
class TestNode
{
  private:
  ros::NodeHandle nh;
  Eigen::Matrix3f mat;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TestNode();
    void copy(const Eigen::Ref<const Eigen::MatrixXf>& m);
    Eigen::Vector4f multiplyQuaternions(Eigen::Vector4f q1, Eigen::Vector4f q2);
    Eigen::Matrix4f quaternionMatrix(Eigen::Vector4f q);
};
}

#endif