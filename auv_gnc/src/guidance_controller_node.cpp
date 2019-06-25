#include "auv_gnc/guidance_controller.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "guidance_controller");
  ros::NodeHandle nh("~");
  auv_gnc::GuidanceController gcon(nh);

  ros::Rate rate(50);
  while (ros::ok)
  {
    ros::spinOnce();
    gcon.runController();
    rate.sleep();
  }
  return 0;
}