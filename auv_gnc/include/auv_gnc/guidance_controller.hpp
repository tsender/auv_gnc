#ifndef GUIDANCE_CONTROLLER
#define GUIDANCE_CONTROLLER

#include "auv_core/auv_core_headers.hpp"
#include "auv_core/eigen_ros.hpp"
#include "auv_control/auv_lqr.hpp"

#include "auv_guidance/basic_trajectory.hpp"
#include "auv_guidance/waypoint.hpp"

#include "auv_msgs/SixDoF.h"
#include "auv_msgs/Thrust.h"
#include "auv_msgs/Trajectory.h"
#include "auv_msgs/TrajectoryGeneratorAction.h"

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "auv_control/LQRGainsConfig.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <boost/thread.hpp>
#include <vector>
#include "math.h"

namespace acc = auv_core::constants;

namespace auv_gnc
{
class GuidanceController
{
private:
   // AUV Model Parameters
   int numActiveThrusters_;
   std::vector<std::string> activeThrusterNames_, inactiveThrusterNames_;
   std::string auvConfigFile_;
   YAML::Node auvConfig_;
   auv_core::auvParameters *auvParams_;

   // Dynamic Reconfigure Setup
   typedef dynamic_reconfigure::Server<auv_control::LQRGainsConfig> DynamicReconfigServer;
   boost::shared_ptr<DynamicReconfigServer> paramReconfigServer_;
   DynamicReconfigServer::CallbackType paramReconfigCB_;
   boost::recursive_mutex paramReconfigMutex_;
   bool dynamicReconfigInit_;

   // LQR Parameters
   auv_control::AUVLQR *auvLQR_;
   auv_core::Matrix18d Q_Aug_;
   auv_core::Matrix8d R_;
   std::vector<double> Q_Diag_, Q_DiagIntegrator_, R_Diag_;
   bool initLQR_;
   bool enableLQRIntegrator_;
   double loopRate_;

   // Trajectory Generator Parameters
   auv_core::auvConstraints *auvConstraints_;
   auv_msgs::Trajectory desiredTrajectory_;
   auv_guidance::Waypoint *startWaypoint_, *endWaypoint_;
   auv_guidance::BasicTrajectory *basicTrajectory_;
   auv_core::Vector13d state_;
   auv_core::Vector13d ref_;
   auv_core::Vector6d accel_;
   Eigen::Vector3d linearAccel_;
   Eigen::Quaterniond quaternion_;
   int tgenType_;
   bool tgenInit_, newTrajectory_;
   ros::Time startTime_;
   auv_core::Vector8d thrust_;

   // ROS Node Parameters
   ros::NodeHandle nh_;
   ros::Subscriber sixDofSub_;
   ros::Publisher thrustPub_;
   std::string subTopic_, pubTopic_, actionName_;
   double trajectoryDuration_;
   bool resultMessageSent_;

   typedef actionlib::SimpleActionServer<auv_msgs::TrajectoryGeneratorAction> TGenActionServer;
   typedef std::shared_ptr<TGenActionServer> TGenActionServerPtr;
   TGenActionServerPtr tgenActionServer_;

   // Private Methods
   void loadAUVParams();
   void loadAUVConstraints();
   void loadAUVLQR();
   
   void initDynamicReconfigure();
   void updateDynamicReconfig(auv_control::LQRGainsConfig config);
   void dynamicReconfigCB(auv_control::LQRGainsConfig &config, uint32_t levels);

   void sixDofCB(const auv_msgs::SixDoF::ConstPtr &state);
   void tgenActionGoalCB();
   void tgenActionPreemptCB();
   bool isActionServerActive();
   bool isTrajectoryTypeValid(int type);
   void initNewTrajectory();
   void publishThrustMessage();

public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   GuidanceController(ros::NodeHandle nh);
   void runController();
};
} // namespace auv_gnc

#endif