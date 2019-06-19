#include "auv_gnc/trans_ekf.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "trans_ekf");
  ros::NodeHandle nh("~");
  auv_gnc::TransEKF transEKF(nh);

  ros::spin();
  return 0;
}