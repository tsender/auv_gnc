#ifndef POSE_EDKF_INTERFACE
#define POSE_EDKF_INTERFACE

#include "auv_gnc/auv_math_lib.hpp"
#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include "math.h"
#include <sstream>
#include <cppad/cppad.hpp>

using namespace Eigen;
using namespace std;
//using namespace CppAD;
using CppAD::AD;
typedef Matrix< AD<double> , Dynamic, Dynamic > ADMatrixXd;
typedef Matrix< AD<double> , Dynamic, 1 > ADVectorXd;

namespace AUV_GNC
{
class TestNode
{
  private:
  ros::NodeHandle nh;
  Matrix3f mat;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TestNode();
    void copy(const Ref<const MatrixXf>& m);
    template <typename T>
    int sign(T x);
};
}

#endif