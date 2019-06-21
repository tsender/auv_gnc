#ifndef AUV_CONTROLLER
#define AUV_CONTROLLER

#include "auv_guidance/basic_trajectory.hpp"
#include "auv_control/auv_model.hpp"
#include "auv_msgs/SixDoF.h"
#include "auv_msgs/Thrust.h"
#include <ros/ros.h>
#include "math.h"
#include <vector>

namespace auv_gnc
{

class AUVController
{
 private:

   typedef actionlib::SimpleActionServer<auv_msgs::TrajectoryAction> TrajectoryActionServer;
   typedef std::shared_ptr<TrajectoryActionServer> TrajectoryActionServerPtr;

   bool init_;
   double dt;
   ros::Time timeLast_;

   ros::NodeHandle nh_;
   ros::Subscriber sixDoFSub_;
   std::string subTopic_, pubTopic_;

 public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   static const int STATE_POS = 0;
   static const int STATE_VEL = 3;
   static const int STATE_ACCEL = 6;

   AUVController(ros::NodeHandle nh);
   void initEKF();
   void sixDofCB(const auv_msgs::SixDoF::ConstPtr &state);
};
} // namespace auv_gnc

#endif