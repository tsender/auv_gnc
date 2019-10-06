#include "auv_gnc/guidance_controller.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "guidance_controller");
   ros::NodeHandle nh("~");
   auv_gnc::GuidanceController gcon(nh);

   double loopRate = 0;
   nh.param("loop_rate", loopRate, 50.0);
   ros::Rate rate(loopRate);
   while (ros::ok())
   {
      ros::spinOnce();
      gcon.runController();
      rate.sleep();
   }
   return 0;
}