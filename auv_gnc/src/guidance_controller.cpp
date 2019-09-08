#include "auv_gnc/guidance_controller.hpp"

namespace auv_gnc
{
GuidanceController::GuidanceController(ros::NodeHandle nh)
{
    nh_ = nh;

    // AUV Model Variables
    nh_.param("auv_model_path", auvConfigFile_, std::string("none"));
    if (auvConfigFile_ == std::string("none"))
    {
        ROS_ERROR("AUD Model not specified. Shutting down.");
        ros::shutdown();
    }
    auvConfig_ = YAML::LoadFile(auvConfigFile_);

    // LQR Variables
    nh_.param("Q_diag", Qdiag_, std::vector<double>(0));
    nh_.param("Q_diag_integral", QdiagIntegral_, std::vector<double>(0));
    nh_.param("R_diag", Rdiag_, std::vector<double>(0));
    nh_.param("enable_LQR_integral", enableLQRIntegral_, false);

    GuidanceController::initAUVModel();

    // Trajectory Generator Limits
    double maxXYDistance, maxZDistance, maxPathInclination;
    double maxXVel, maxYVel, maxZVel, maxRotVel;
    double maxXAccel, maxYAccel, maxZAccel, maxRotAccel;
    double xyzJerk, xyzClosingJerk;
    double rotJerk, rotClosingJerk;
    double closingTolXYZ, closingTolRot;

    nh_.param("max_xy_distance", maxXYDistance, 3.0);               // [m]
    nh_.param("max_z_distance", maxZDistance, 1.0);                 // [m]
    nh_.param("max_path_inclination", maxPathInclination, 1.40);    // [rad]
    nh_.param("closing_tolerance/xyz", closingTolXYZ, 0.1);         // [m]
    nh_.param("closing_tolerance/rotational", closingTolRot, 6.0);  // [rad]

    nh_.param("max_velocity/x", maxXVel, 0.75);                     // [m/s]
    nh_.param("max_velocity/y", maxYVel, 0.5);                      // [m/s]
    nh_.param("max_velocity/z", maxZVel, 0.3);                      // [m/s]
    nh_.param("max_velocity/rotational", maxRotVel, 1.57);          // [rad/s]

    nh_.param("max_accel/x", maxXAccel, 0.4);                       // [m/s^2]
    nh_.param("max_accel/y", maxYAccel, 0.4);                       // [m/s^2]
    nh_.param("max_accel/z", maxZAccel, 0.2);                       // [m/s^2]
    nh_.param("max_accel/rotational", maxRotAccel, 3.14);           // [rad/s^2]

    nh_.param("jerk/xyz/nominal", xyzJerk, 0.4);                    // [m/s^3]
    nh_.param("jerk/xyz/closing", xyzClosingJerk, 1.0);             // [m/s^3]
    nh_.param("jerk/rotational/nominal", rotJerk, 5.0);             // [rad/s^3]
    nh_.param("jerk/rotational/closing", rotClosingJerk, 6.0);      // [rad/s^3]

    tgenLimits_ = new auv_guidance::TGenLimits(maxXYDistance, maxZDistance, maxPathInclination, closingTolXYZ, closingTolRot,
                                               maxXVel, maxYVel, maxZVel, maxRotVel, maxXAccel, maxYAccel, maxZAccel, maxRotAccel,
                                               xyzJerk, xyzClosingJerk, rotJerk, rotClosingJerk);

    // Pubs, Subs, and Action Servers
    nh_.param("subscriber_topic", subTopic_, std::string("/auv_gnc/trans_ekf/six_dof"));
    nh_.param("publisher_topic", pubTopic_, std::string("/auv_gnc/controller/thrust"));
    nh_.param("action_name", actionName_, std::string("/auv_gnc/controller/check_for_trajectory"));

    sixDofSub_ = nh_.subscribe<auv_msgs::SixDoF>(subTopic_, 1, &GuidanceController::sixDofCB, this);
    thrustPub_ = nh_.advertise<auv_msgs::Thrust>(pubTopic_, 1, this);

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

    // Initialize action server
    tgenActionServer_.reset(new TGenActionServer(nh_, actionName_, false));
    tgenActionServer_->registerGoalCallback(boost::bind(&GuidanceController::tgenActionGoalCB, this));
    tgenActionServer_->registerPreemptCallback(boost::bind(&GuidanceController::tgenActionPreemptCB, this));
    tgenActionServer_->start();

    ROS_INFO("Guidance Controller initialized");
}

/**
 * \brief Initialize AUV model from parameters
 */
void GuidanceController::initAUVModel()
{
    // Fg and Fb
    double Fg = fabs(auvConfig_["Fg"].as<double>());
    double Fb = fabs(auvConfig_["Fb"].as<double>());

    // Center of Buoyancy (relative to the center of mass)
    Eigen::Vector3d CoB = Eigen::Vector3d::Zero();
    CoB[0] = auvConfig_["center_of_buoyancy"][0].as<double>(); // X
    CoB[1] = auvConfig_["center_of_buoyancy"][1].as<double>(); // Y
    CoB[2] = auvConfig_["center_of_buoyancy"][2].as<double>(); // Z

    // Center of Mass (relative to the reference point)
    Eigen::Vector3d CoM = Eigen::Vector3d::Zero();
    CoM[0] = auvConfig_["center_of_mass"][0].as<double>(); // X
    CoM[1] = auvConfig_["center_of_mass"][1].as<double>(); // Y
    CoM[2] = auvConfig_["center_of_mass"][2].as<double>(); // Z

    // Inertia
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
    double ixx = fabs(auvConfig_["inertia"]["Ixx"].as<double>());
    double iyy = fabs(auvConfig_["inertia"]["Iyy"].as<double>());
    double izz = fabs(auvConfig_["inertia"]["Izz"].as<double>());
    double ixy = auvConfig_["inertia"]["Ixy"].as<double>();
    double ixz = auvConfig_["inertia"]["Ixz"].as<double>();
    double iyz = auvConfig_["inertia"]["Iyz"].as<double>();
    inertia(0, 0) = ixx;
    inertia(0, 1) = ixy;
    inertia(0, 2) = ixz;
    inertia(1, 0) = ixy;
    inertia(1, 1) = iyy;
    inertia(1, 2) = iyz;
    inertia(2, 0) = ixz;
    inertia(2, 1) = iyz;
    inertia(2, 2) = izz;

    // Drag
    auv_control::Matrix62d dragCoeffs;
    dragCoeffs.setZero();
    for (int i = 0; i < 3; i++)
    {
        dragCoeffs(i, 0) = fabs(auvConfig_["translational_drag"]["linear"][i].as<double>());
        dragCoeffs(i + 3, 0) = fabs(auvConfig_["translational_drag"]["quadratic"][i].as<double>());
        dragCoeffs(i, 1) = fabs(auvConfig_["rotational_drag"]["linear"][i].as<double>());
        dragCoeffs(i + 3, 1) = fabs(auvConfig_["rotational_drag"]["quadratic"][i].as<double>());
    }

    // Thrusters
    numActiveThrusters_ = 0;
    activeThrusterNames_.clear();
    inactiveThrusterNames_.clear();

    int numThrusters = auvConfig_["thrusters"].size();
    numThrusters = std::max(numThrusters, 8); // Cap thrusters at 8
    std::vector<bool> thrustersEnabled;
    thrustersEnabled.clear();

    for (int i = 0; i < numThrusters; i++)
    {
        bool enabled = auvConfig_["thrusters"][i]["enable"].as<bool>();
        thrustersEnabled.push_back(enabled);
        std::string name = auvConfig_["thrusters"][i]["name"].as<std::string>();
        if (enabled)
        {
            numActiveThrusters_++;
            activeThrusterNames_.push_back(name);
        }
        else
            inactiveThrusterNames_.push_back(name);
    }

    // Each COLUMN contains a thruster's data (x, y, z, yaw, pitch)
    auv_control::Matrix58d thrusterData;
    thrusterData.setZero();
    int col = 0;
    for (int i = 0; i < numThrusters; i++)
    {
        if (thrustersEnabled[i])
        {
            for (int j = 0; j < 5; j++)
                if (j < 3)
                    thrusterData(j, col) = auvConfig_["thrusters"][i]["pose"][j].as<double>() - CoM(j);
                else
                    thrusterData(j, col) = auvConfig_["thrusters"][i]["pose"][j].as<double>();
            col++;
        }
    }

    auvModel_ = new auv_control::AUVModel(Fg, Fb, CoB, inertia, dragCoeffs, thrusterData, numActiveThrusters_);

    // LQR Cost Matrices
    auv_control::Matrix12d Q;
    auv_control::Matrix18d Qaug;
    auv_control::Matrix8d R;
    Q.setZero();
    Qaug.setZero();
    R.setZero();

    for (int i = 0; i < 12; i++)
    {
        if (i < 8)
            R(i, i) = fabs(Rdiag_[i]);
        Q(i, i) = fabs(Qdiag_[i]);
    }

    if (!enableLQRIntegral_)
    {
        auvModel_->setLQRCostMatrices(Q, R);
    }
    else
    {
        Qaug.block<12, 12>(0, 0) = Q;
        for (int i = 0; i < 6; i++)
        {
            Qaug(12 + i, 12 + i) = fabs(QdiagIntegral_[i]);
        }
        auvModel_->setLQRIntegralCostMatrices(Qaug, R);
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

        thrust_ = auvModel_->computeLQRThrust(state_, ref_, accel_);
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
        basicTrajectory_ = new auv_guidance::BasicTrajectory(startWaypoint_, endWaypoint_, tgenLimits_);
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
