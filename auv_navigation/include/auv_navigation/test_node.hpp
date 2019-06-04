#ifndef POSE_EDKF_INTERFACE
#define POSE_EDKF_INTERFACE

#include "auv_navigation/auv_math_lib.hpp"
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
typedef Matrix<float, 4, 1> Vector4f;
typedef Matrix<float, 4, 4> Matrix4f;

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
    Vector4f multiplyQuaternions(Vector4f q1, Vector4f q2);
    Matrix4f quaternionMatrix(Vector4f q);
};
}

#endif