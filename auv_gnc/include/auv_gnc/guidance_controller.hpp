#ifndef GUIDANCE_CONTROLLER
#define GUIDANCE_CONTROLLER

#include "auv_control/auv_model.hpp"
#include "auv_core/constants.hpp"
#include "auv_guidance/basic_trajectory.hpp"
#include "auv_guidance/tgen_limits.hpp"
#include "auv_guidance/waypoint.hpp"
#include "auv_msgs/SixDoF.h"
#include "auv_msgs/Thrust.h"
#include "auv_msgs/Trajectory.h"
#include "auv_msgs/TrajectoryGeneratorAction.h"

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include "eigen_conversions/eigen_msg.h"

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
  auv_control::AUVModel *auvModel_;

  // LQR Parameters
  std::vector<double> Qdiag_, Rdiag_;
  auv_control::Matrix12d Q_;
  auv_control::Matrix8d R_;

  // Trajectory Generator Parameters
  auv_msgs::Trajectory desiredTrajectory_;
  int tgenType_;
  bool tgenInit_, newTrajectory_;
  Eigen::Quaterniond quaternion_;
  ros::Time timeStart_;
  auv_control::Vector8d thrust_;

  auv_guidance::TGenLimits *tgenLimits_;
  auv_guidance::Vector13d state_;
  Eigen::Vector3d linearAccel_;
  auv_guidance::Waypoint *startWaypoint_, *endWaypoint_;
  auv_guidance::BasicTrajectory *basicTrajectory_;

  // ROS Parameters
  ros::NodeHandle nh_;
  ros::Subscriber sixDoFSub_;
  ros::Publisher thrustPub_;
  std::string subTopic_, pubTopic_, actionName_;
  double trajectoryDuration_;
  bool resultMessageSent_;

  typedef actionlib::SimpleActionServer<auv_msgs::TrajectoryGeneratorAction> TGenActionServer;
  typedef std::shared_ptr<TGenActionServer> TGenActionServerPtr;
  TGenActionServerPtr tgenActionServer_;

  // Private Methods
  void initAUVModel();
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
}  // namespace auv_gnc

#endif