#include "auv_gnc/guidance_controller.hpp"

namespace auv_gnc
{
GuidanceController::GuidanceController(ros::NodeHandle nh)
{
   nh_ = nh;

   // AUV Model Variables
   nh_.param("auv_config_path", auvConfigFile_, std::string("none"));
   if (auvConfigFile_ == std::string("none"))
   {
      ROS_ERROR("AUD Model not specified. Shutting down.");
      ros::shutdown();
   }
   auvConfig_ = YAML::LoadFile(auvConfigFile_);
   nh_.param("loop_rate", loopRate_, 50.0);

   auvParams_ = new auv_core::auvParameters;
   auvConstraints_ = new auv_core::auvConstraints;

   // Initialize variables
   state_.setZero();
   ref_.setZero();
   accel_.setZero();
   linearAccel_.setZero();
   thrust_.setZero();
   quaternion_.setIdentity();

   tgenType_ = 0;
   tgenInit_ = false;
   newTrajectory_ = false;
   resultMessageSent_ = false;
   trajectoryDuration_ = 0;

   // Init dynamic reconfigure
   dynamicReconfigInit_ = false;
   initLQR_ = false;
   GuidanceController::initDynamicReconfigure();

   GuidanceController::loadAUVParams();
   GuidanceController::loadAUVConstraints();
   GuidanceController::loadAUVLQR();

   // Pubs, Subs, and Action Servers
   nh_.param("subscriber_topic", subTopic_, std::string("/auv_gnc/trans_ekf/six_dof"));
   nh_.param("publisher_topic", pubTopic_, std::string("/auv_gnc/controller/thrust"));
   nh_.param("action_name", actionName_, std::string("/auv_gnc/controller/check_for_trajectory"));

   sixDofSub_ = nh_.subscribe<auv_msgs::SixDoF>(subTopic_, 1, &GuidanceController::sixDofCB, this);
   thrustPub_ = nh_.advertise<auv_msgs::Thrust>(pubTopic_, 1, this);

   // Initialize action server
   tgenActionServer_.reset(new TGenActionServer(nh_, actionName_, false));
   tgenActionServer_->registerGoalCallback(boost::bind(&GuidanceController::tgenActionGoalCB, this));
   tgenActionServer_->registerPreemptCallback(boost::bind(&GuidanceController::tgenActionPreemptCB, this));
   tgenActionServer_->start();

   ROS_INFO("Guidance Controller initialized");
}

/**
 * \brief Collect AUV parameters
 */
void GuidanceController::loadAUVParams()
{
   // Mass and Fb
   auvParams_->mass = fabs(auvConfig_["model"]["mass"].as<double>());
   auvParams_->Fb = fabs(auvConfig_["model"]["Fb"].as<double>());

   // Center of Buoyancy (relative to the center of mass)
   Eigen::Vector3d CoB = Eigen::Vector3d::Zero();
   CoB[0] = auvConfig_["model"]["center_of_buoyancy"][0].as<double>(); // X
   CoB[1] = auvConfig_["model"]["center_of_buoyancy"][1].as<double>(); // Y
   CoB[2] = auvConfig_["model"]["center_of_buoyancy"][2].as<double>(); // Z
   auvParams_->cob = CoB;

   // Center of Mass (relative to the reference point)
   Eigen::Vector3d CoM = Eigen::Vector3d::Zero();
   CoM[0] = auvConfig_["model"]["center_of_mass"][0].as<double>(); // X
   CoM[1] = auvConfig_["model"]["center_of_mass"][1].as<double>(); // Y
   CoM[2] = auvConfig_["model"]["center_of_mass"][2].as<double>(); // Z

   // Inertia
   Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
   double ixx = fabs(auvConfig_["model"]["inertia"]["Ixx"].as<double>());
   double iyy = fabs(auvConfig_["model"]["inertia"]["Iyy"].as<double>());
   double izz = fabs(auvConfig_["model"]["inertia"]["Izz"].as<double>());
   double ixy = auvConfig_["model"]["inertia"]["Ixy"].as<double>();
   double ixz = auvConfig_["model"]["inertia"]["Ixz"].as<double>();
   double iyz = auvConfig_["model"]["inertia"]["Iyz"].as<double>();
   inertia(0, 0) = ixx;
   inertia(0, 1) = -ixy;
   inertia(0, 2) = -ixz;
   inertia(1, 0) = -ixy;
   inertia(1, 1) = iyy;
   inertia(1, 2) = -iyz;
   inertia(2, 0) = -ixz;
   inertia(2, 1) = -iyz;
   inertia(2, 2) = izz;
   auvParams_->inertia = inertia;

   // Drag
   auv_core::Matrix62d dragCoeffs;
   dragCoeffs.setZero();
   for (int i = 0; i < 3; i++)
   {
      // Col 0: translational
      dragCoeffs(i, 0) = fabs(auvConfig_["model"]["translational_drag"]["linear"][i].as<double>()); // Rows 0-2: linear terms
      dragCoeffs(i + 3, 0) = fabs(auvConfig_["model"]["translational_drag"]["quadratic"][i].as<double>()); // Rows 3-4: quadratic terms
      // Col 1: rotational
      dragCoeffs(i, 1) = fabs(auvConfig_["model"]["rotational_drag"]["linear"][i].as<double>()); // Rows 0-2: linear terms
      dragCoeffs(i + 3, 1) = fabs(auvConfig_["model"]["rotational_drag"]["quadratic"][i].as<double>()); // Rows 3-4: quadratic terms
   }
   auvParams_->dragCoeffs = dragCoeffs;

   // Thrusters
   numActiveThrusters_ = 0;
   activeThrusterNames_.clear();
   inactiveThrusterNames_.clear();

   int numThrusters = auvConfig_["model"]["thrusters"].size();
   numThrusters = std::max(numThrusters, 8); // Cap thrusters at 8
   std::vector<bool> thrustersEnabled;
   thrustersEnabled.clear();

   for (int i = 0; i < numThrusters; i++)
   {
      bool enabled = auvConfig_["model"]["thrusters"][i]["enable"].as<bool>();
      thrustersEnabled.push_back(enabled);
      std::string name = auvConfig_["model"]["thrusters"][i]["name"].as<std::string>();
      if (enabled)
      {
         numActiveThrusters_++;
         activeThrusterNames_.push_back(name);
      }
      else
         inactiveThrusterNames_.push_back(name);
   }

   // Each COLUMN contains a thruster's data (x, y, z, yaw, pitch)
   auv_core::Matrix58d thrusterData;
   thrusterData.setZero();
   int col = 0;
   for (int i = 0; i < numThrusters; i++)
   {
      if (thrustersEnabled[i])
      {
         for (int j = 0; j < 5; j++)
            if (j < 3)
               thrusterData(j, col) = auvConfig_["model"]["thrusters"][i]["pose"][j].as<double>() - CoM(j);
            else
               thrusterData(j, col) = auvConfig_["model"]["thrusters"][i]["pose"][j].as<double>();
         col++;
      }
   }

   auvParams_->numThrusters = numActiveThrusters_;
   auvParams_->thrusterData = thrusterData;
   ROS_INFO("Guidance Controller: Loaded parameters");
}

/**
 * \brief Load AUV constraints
 */
void GuidanceController::loadAUVConstraints()
{
   auvConstraints_->maxXYDistance = auvConfig_["constraints"]["simultaneous_trajectory_threshold"]["max_xy_distance"].as<double>();
   auvConstraints_->maxZDistance = auvConfig_["constraints"]["simultaneous_trajectory_threshold"]["max_z_distance"].as<double>();
   auvConstraints_->maxAlignInclination = auvConfig_["constraints"]["max_align_inclination"].as<double>();

   Eigen::Vector3d maxTransVel = Eigen::Vector3d::Zero();
   auvConstraints_->maxTransVel(0) = auvConfig_["constraints"]["max_velocity"]["x"].as<double>();
   auvConstraints_->maxTransVel(1) = auvConfig_["constraints"]["max_velocity"]["y"].as<double>();
   auvConstraints_->maxTransVel(2) = auvConfig_["constraints"]["max_velocity"]["z"].as<double>();
   //auvConstraints_->maxTransVel = maxTransVel;
   auvConstraints_->maxRotVel = auvConfig_["constraints"]["max_velocity"]["rot"].as<double>();

   Eigen::Vector3d maxTransAccel = Eigen::Vector3d::Zero();
   auvConstraints_->maxTransAccel(0) = auvConfig_["constraints"]["max_accel"]["x"].as<double>();
   auvConstraints_->maxTransAccel(1) = auvConfig_["constraints"]["max_accel"]["y"].as<double>();
   auvConstraints_->maxTransAccel(2) = auvConfig_["constraints"]["max_accel"]["z"].as<double>();
   //auvConstraints_->maxTransAccel = maxTransAccel;
   auvConstraints_->maxRotAccel = auvConfig_["constraints"]["max_accel"]["rot"].as<double>();

   auvConstraints_->transJerk = auvConfig_["constraints"]["jerk"]["trans"].as<double>();
   auvConstraints_->rotJerk = auvConfig_["constraints"]["jerk"]["rot"].as<double>();
   ROS_INFO("Guidance Controller: Loaded constraints");
}

/**
 * \brief Load AUV LQR parameters
 */
void GuidanceController::loadAUVLQR()
{
   // LQR Cost Matrices
   Q_Aug_.setZero();
   R_.setZero();

   bool enableLQRIntegrator = auvConfig_["LQR"]["enable_integrator"].as<bool>();
   for (int i = 0; i < 12; i++)
   {
      if (i < 6)
         Q_Aug_(i + 12, i + 12) = fabs(auvConfig_["LQR"]["Q_diag_integrator"][i].as<double>());
      if (i < 8)
         R_(i, i) = fabs(auvConfig_["LQR"]["R_diag"][i].as<double>());
      Q_Aug_(i, i) = fabs(auvConfig_["LQR"]["Q_diag"][i].as<double>());
   }

   auvLQR_ = new auv_control::AUVLQR(auvParams_, 1 / loopRate_);
   auvLQR_->setCostMatrices(Q_Aug_, R_);
   auvLQR_->setIntegrator(enableLQRIntegrator);
   initLQR_ = true;

   // Initialize dynamic reconfig values
   auv_control::LQRGainsConfig config;
   config.Q_x = Q_Aug_(0, 0);
   config.Q_y = Q_Aug_(1, 1);
   config.Q_z = Q_Aug_(2, 2);
   config.Q_u = Q_Aug_(3, 3);
   config.Q_v = Q_Aug_(4, 4);
   config.Q_w = Q_Aug_(5, 5);
   config.Q_q1 = Q_Aug_(6, 6);
   config.Q_q2 = Q_Aug_(7, 7);
   config.Q_q3 = Q_Aug_(8, 8);
   config.Q_p = Q_Aug_(9, 9);
   config.Q_q = Q_Aug_(10, 10);
   config.Q_r = Q_Aug_(11, 11);
   config.Q_Int_x = Q_Aug_(12, 12);
   config.Q_Int_y = Q_Aug_(13, 13);
   config.Q_Int_z = Q_Aug_(14, 14);
   config.Q_Int_q1 = Q_Aug_(15, 15);
   config.Q_Int_q2 = Q_Aug_(16, 16);
   config.Q_Int_q3 = Q_Aug_(17, 17);
   config.R1 = R_(0, 0);
   config.R2 = R_(1, 1);
   config.R3 = R_(2, 2);
   config.R4 = R_(3, 3);
   config.R5 = R_(4, 4);
   config.R6 = R_(5, 5);
   config.R7 = R_(6, 6);
   config.R8 = R_(7, 7);
   GuidanceController::updateDynamicReconfig(config);

   ROS_INFO("Guidance Controller: Initialized LQR controller");
}

/**
 * \brief Initializes dynamic reconfigure for LQR Q matrix
 */
void GuidanceController::initDynamicReconfigure()
{
   // Reset server
   paramReconfigServer_.reset(new DynamicReconfigServer(paramReconfigMutex_, nh_));
   dynamicReconfigInit_ = true;

   // Now, we set the callback
   paramReconfigCB_ = boost::bind(&GuidanceController::dynamicReconfigCB, this, _1, _2);
   paramReconfigServer_->setCallback(paramReconfigCB_);
   ROS_INFO("GuidanceController: Initialized dynamic reconfigure");
}

void GuidanceController::updateDynamicReconfig(auv_control::LQRGainsConfig config)
{
   // Make sure dynamic reconfigure is initialized
   if (!dynamicReconfigInit_)
      return;

   // Set starting values, using a shared mutex with dynamic reconfig
   paramReconfigMutex_.lock();
   paramReconfigServer_->updateConfig(config);
   paramReconfigMutex_.unlock();
}

// Callback for dynamic reconfigure
void GuidanceController::dynamicReconfigCB(auv_control::LQRGainsConfig &config, uint32_t levels)
{
   if (!initLQR_)
      return;

   if (levels == 0) // Level 0: State cost matrix
   {
      Q_Aug_(0, 0) = config.Q_x;
      Q_Aug_(1, 1) = config.Q_y;
      Q_Aug_(2, 2) = config.Q_z;
      Q_Aug_(3, 3) = config.Q_u;
      Q_Aug_(4, 4) = config.Q_v;
      Q_Aug_(5, 5) = config.Q_w;
      Q_Aug_(6, 6) = config.Q_q1;
      Q_Aug_(7, 7) = config.Q_q2;
      Q_Aug_(8, 8) = config.Q_q3;
      Q_Aug_(9, 9) = config.Q_p;
      Q_Aug_(10, 10) = config.Q_q;
      Q_Aug_(11, 11) = config.Q_r;

      Q_Aug_(12, 12) = config.Q_Int_x;
      Q_Aug_(13, 13) = config.Q_Int_y;
      Q_Aug_(14, 14) = config.Q_Int_z;
      Q_Aug_(15, 15) = config.Q_Int_q1;
      Q_Aug_(16, 16) = config.Q_Int_q2;
      Q_Aug_(17, 17) = config.Q_Int_q3;
      auvLQR_->setCostMatrixQ(Q_Aug_);
   }
   else // Level 1: Input cost matrix
   {
      R_(0, 0) = config.R1;
      R_(1, 1) = config.R2;
      R_(2, 2) = config.R3;
      R_(3, 3) = config.R4;
      R_(4, 4) = config.R5;
      R_(5, 5) = config.R6;
      R_(6, 6) = config.R7;
      R_(7, 7) = config.R8;
      auvLQR_->setCostMatrixR(R_);
   }
}

/**
 * \brief Process new six DoF data thru EKF
 */
void GuidanceController::sixDofCB(const auv_msgs::SixDoF::ConstPtr &state)
{
   // Inertial Position, expressed in Inertial-frame
   state_(acc::STATE_XI) = state->pose.position.x;
   state_(acc::STATE_YI) = state->pose.position.y;
   state_(acc::STATE_ZI) = state->pose.position.z;

   auv_core::eigen_ros::quaternionMsgToEigen(state->pose.orientation, quaternion_);

   // Inertial Translational Velocity, expressed in Body-frame
   state_(acc::STATE_U) = state->velocity.linear.x;
   state_(acc::STATE_V) = state->velocity.linear.y;
   state_(acc::STATE_W) = state->velocity.linear.z;

   // Quaternion, from Inertial-frame to Body-frame
   state_(acc::STATE_Q0) = state->pose.orientation.w;
   state_(acc::STATE_Q1) = state->pose.orientation.x;
   state_(acc::STATE_Q2) = state->pose.orientation.y;
   state_(acc::STATE_Q3) = state->pose.orientation.z;

   // Rotational Velocity, expressed in Body-frame
   state_(acc::STATE_P) = state->velocity.angular.x;
   state_(acc::STATE_Q) = state->velocity.angular.y;
   state_(acc::STATE_R) = state->velocity.angular.z;

   // Inertial Translational Acceleration, expressed in Body-frame
   auv_core::eigen_ros::vectorMsgToEigen(state->linear_accel, linearAccel_);
}

void GuidanceController::tgenActionGoalCB()
{
   boost::shared_ptr<const auv_msgs::TrajectoryGeneratorGoal> tgenPtr = tgenActionServer_->acceptNewGoal();

   if (GuidanceController::isTrajectoryTypeValid(tgenPtr->trajectory.type))
   {
      desiredTrajectory_ = tgenPtr->trajectory;
      tgenType_ = desiredTrajectory_.type;
      newTrajectory_ = true;
      resultMessageSent_ = false;

      if (!tgenInit_)
         tgenInit_ = true;
   }
   else
   {
      auv_msgs::TrajectoryGeneratorResult result;
      result.completed = false;
      tgenActionServer_->setAborted(result);
      resultMessageSent_ = false;
   }
}

void GuidanceController::tgenActionPreemptCB()
{
   tgenActionServer_->setPreempted();
   tgenInit_ = false;
   thrust_.setZero();
   resultMessageSent_ = false;
   GuidanceController::publishThrustMessage();
}

bool GuidanceController::isActionServerActive()
{
   return (tgenActionServer_->isActive() && !tgenActionServer_->isPreemptRequested());
}

/**
 * @param type Type of trajectory
 * \brief Returns true if trajectory type is valid, false otherwise
 */
bool GuidanceController::isTrajectoryTypeValid(int type)
{
   if (type == auv_msgs::Trajectory::BASIC_ABS_XYZ)
      return true;
   else if (type == auv_msgs::Trajectory::BASIC_REL_XYZ)
      return true;
   return false;
}

void GuidanceController::runController()
{
   if (!tgenInit_)
      return;

   if (newTrajectory_) // Initialize new trajectory once
   {
      GuidanceController::initNewTrajectory();
   }
   else
   {
      double evalTime = ros::Time::now().toSec() - startTime_.toSec();

      if (tgenType_ == auv_msgs::Trajectory::BASIC_ABS_XYZ || tgenType_ == auv_msgs::Trajectory::BASIC_ABS_XYZ)
      {
         ref_ = basicTrajectory_->computeState(evalTime);
         accel_ = basicTrajectory_->computeAccel(evalTime);
         //ROS_INFO("Time in Trajectory: %f", dt);
         //std::cout << "Reference state: " << std::endl << ref << std::endl; // Debug
         //std::cout << "Accel state: " << std::endl << accel << std::endl; // Debug
      }

      if (evalTime > trajectoryDuration_ && !resultMessageSent_)
      {
         resultMessageSent_ = true;
         auv_msgs::TrajectoryGeneratorResult result;
         result.completed = true;
         tgenActionServer_->setSucceeded(result);
      }

      thrust_ = auvLQR_->computeThrust(state_, ref_, accel_);
      GuidanceController::publishThrustMessage();
   }
}

/**
 * \brief Initialize new trajectory
 */
void GuidanceController::initNewTrajectory()
{
   ROS_INFO("GuidanceController: Initializing new trajectory.");
   newTrajectory_ = false;
   startTime_ = ros::Time::now();

   Eigen::Vector3d zero3d = Eigen::Vector3d::Zero();
   Eigen::Vector3d posIStart = zero3d;
   Eigen::Vector3d velIStart = zero3d;
   Eigen::Vector3d accelIStart = zero3d;

   // Inertial position, velocity, and accel expressed in I-frame
   posIStart = state_.segment<3>(acc::STATE_XI);
   velIStart = quaternion_ * state_.segment<3>(acc::STATE_U);
   accelIStart = quaternion_ * linearAccel_;

   startWaypoint_ = new auv_guidance::Waypoint(posIStart, velIStart, accelIStart, quaternion_, state_.segment<3>(acc::STATE_P));

   if (tgenType_ == auv_msgs::Trajectory::BASIC_ABS_XYZ || tgenType_ == auv_msgs::Trajectory::BASIC_REL_XYZ)
   {
      Eigen::Vector3d posIEnd = zero3d;
      Eigen::Quaterniond quatEnd;

      auv_core::eigen_ros::pointMsgToEigen(desiredTrajectory_.pose.position, posIEnd);
      auv_core::eigen_ros::quaternionMsgToEigen(desiredTrajectory_.pose.orientation, quatEnd);

      if (tgenType_ == auv_msgs::Trajectory::BASIC_REL_XYZ)
         posIEnd = (quaternion_ * posIStart) + posIEnd;

      endWaypoint_ = new auv_guidance::Waypoint(posIEnd, zero3d, zero3d, quatEnd, zero3d);
      basicTrajectory_ = new auv_guidance::BasicTrajectory(auvConstraints_, startWaypoint_, endWaypoint_);
      trajectoryDuration_ = basicTrajectory_->getTime();
   }
}

void GuidanceController::publishThrustMessage()
{
   auv_msgs::Thrust thrustMsg;

   for (int i = 0; i < activeThrusterNames_.size(); i++)
      thrustMsg.names.push_back(activeThrusterNames_[i]);

   for (int i = 0; i < inactiveThrusterNames_.size(); i++)
      thrustMsg.names.push_back(inactiveThrusterNames_[i]);

   for (int i = 0; i < thrust_.rows(); i++)
      thrustMsg.thrusts.push_back(thrust_(i));

   thrustMsg.header.stamp = ros::Time::now();
   thrustPub_.publish(thrustMsg);
}

} // namespace auv_gnc
