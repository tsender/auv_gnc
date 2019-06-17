#ifndef TEST_NODE
#define TEST_NODE

#include "auv_guidance/monotonic_trajectory_time_solver.hpp"
#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include "math.h"
#include <sstream>

using namespace std;

namespace AUVGuidance
{
class TestNode
{
  private:
  ros::NodeHandle nh;
  Eigen::Matrix3f mat;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TestNode();
};
}

#endif