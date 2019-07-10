#include "auv_gnc/guidance_controller.hpp"

namespace auv_gnc
{
GuidanceController::GuidanceController(ros::NodeHandle nh)
{
    nh_ = nh;

    // AUV Model Variables
    nh_.param("auv_model", auvConfigFile_, std::string("none"));
    if (auvConfigFile_ == std::string("none"))
    {
        ROS_ERROR("AUD Model not specified. Shutting down.");
        ros::shutdown();
    }
    auvConfig_ = YAML::LoadFile(auvConfigFile_);

    // LQR Variables
    nh_.param("R_diag", Rdiag_, std::vector<double>(0));
    nh_.param("Q_diag", Qdiag_, std::vector<double>(0));

    GuidanceController::initAUVModel();

    // Trajectory Generator Limits
    double maxXYVelocity, maxXYAccel;
    double maxZVelocity, maxZAccel;
    double maxRotVelocity, maxRotAccel;
    double xyzJerk, xyzClosingJerk;
    double rotJerk, rotClosingJerk;
    double closingTolerance;
    double maxPathInclination;
    double maxXYDistance, maxZDistance;

    nh_.param("max_xy_distance", maxXYDistance, 3.0);            // [m]
    nh_.param("max_z_distance", maxZDistance, 1.0);              // [m]
    nh_.param("max_path_inclination", maxPathInclination, 80.0); // [deg]

    nh_.param("max_xy_velocity", maxXYVelocity, 0.75);          // [m/s]
    nh_.param("max_xy_accel", maxXYAccel, 0.4);                 // [m/s^2]
    nh_.param("max_z_velocity", maxZVelocity, 0.3);             // [m/s]
    nh_.param("max_z_accel", maxZAccel, 0.2);                   // [m/s^2]
    nh_.param("max_rotational_velocity", maxRotVelocity, 1.57); // [rad/s]
    nh_.param("max_rotational_accel", maxRotAccel, 3.14);       // [rad/s^2]

    nh_.param("closing_tolerance", closingTolerance, 0.1);     // [m] or [rad]
    nh_.param("xyz_jerk", xyzJerk, 0.4);                       // [m/s^3]
    nh_.param("xyz_closing_jerk", xyzClosingJerk, 1.0);        // [m/s^3]
    nh_.param("rotational_jerk", rotJerk, 5.0);                // [rad/s^3]
    nh_.param("rotational_closing_jerk", rotClosingJerk, 6.0); // [rad/s^3]

    tgenLimits_ = new auv_guidance::TGenLimits(maxXYDistance, maxZDistance, maxXYVelocity, maxXYAccel,
                                               maxZVelocity, maxZAccel, maxRotVelocity, maxRotAccel,
                                               xyzJerk, xyzClosingJerk, rotJerk, rotClosingJerk,
                                               closingTolerance, maxPathInclination);

    // Pubs, Subs, and Action Servers
    nh_.param("subscriber_topic", subTopic_, std::string("/auv_gnc/trans_ekf/six_dof"));
    nh_.param("publisher_topic", pubTopic_, std::string("/auv_gnc/controller/thrust"));
    nh_.param("action_name", actionName_, std::string("/auv_gnc/controller/check_for_trajectory"));

    sixDoFSub_ = nh_.subscribe<auv_msgs::SixDoF>(subTopic_, 1, &GuidanceController::sixDofCB, this);
    thrustPub_ = nh_.advertise<auv_msgs::Thrust>(pubTopic_, 1, this);

    // Initialize variables
    state_.setZero();
    linearAccel_.setZero();
    thrust_.setZero();

    quaternion_.w() = 1;
    quaternion_.x() = 0;
    quaternion_.y() = 0;
    quaternion_.z() = 0;

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

    // Center of buoyancy
    Eigen::Vector3d CoB = Eigen::Vector3d::Zero();
    CoB[0] = auvConfig_["center_of_buoyancy"][0].as<double>(); // X
    CoB[1] = auvConfig_["center_of_buoyancy"][1].as<double>(); // Y
    CoB[2] = auvConfig_["center_of_buoyancy"][2].as<double>(); // Z

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
    activeThrusters_ = 0;
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
            activeThrusters_++;
            activeThrusterNames_.push_back(name);
        }
        else
            inactiveThrusterNames_.push_back(name);
    }

    // Each COLUMN contains a thruster's info
    auv_control::Matrix58d thrusters;
    thrusters.setZero();
    int col = 0;
    for (int i = 0; i < numThrusters; i++)
    {
        if (thrustersEnabled[i])
        {
            for (int j = 0; j < 5; j++)
                thrusters(j, col) = auvConfig_["thrusters"][i]["pose"][j].as<double>();
            col++;
        }
    }

    auvModel_ = new auv_control::AUVModel(Fg, Fb, CoB, inertia, dragCoeffs, thrusters, activeThrusters_);

    // LQR Cost Matrices
    Q_.setZero();
    R_.setZero();

    for (int i = 0; i < 12; i++)
    {
        if (i < 8)
            R_(i, i) = fabs(Rdiag_[i]);
        Q_(i, i) = fabs(Qdiag_[i]);
    }

    auvModel_->setLQRCostMatrices(Q_, R_);
}

/**
 * \brief Process new six DoF data thru EKF
 */
void GuidanceController::sixDofCB(const auv_msgs::SixDoF::ConstPtr &state)
{
    state_(acc::STATE_XI) = state->pose.position.x;
    state_(acc::STATE_YI) = state->pose.position.y;
    state_(acc::STATE_ZI) = state->pose.position.z;

    tf::quaternionMsgToEigen(state->pose.orientation, quaternion_);

    state_(acc::STATE_U) = state->velocity.linear.x;
    state_(acc::STATE_V) = state->velocity.linear.y;
    state_(acc::STATE_W) = state->velocity.linear.z;

    state_(acc::STATE_Q0) = state->pose.orientation.w;
    state_(acc::STATE_Q1) = state->pose.orientation.x;
    state_(acc::STATE_Q2) = state->pose.orientation.y;
    state_(acc::STATE_Q3) = state->pose.orientation.z;

    state_(acc::STATE_P) = state->velocity.angular.x;
    state_(acc::STATE_Q) = state->velocity.angular.y;
    state_(acc::STATE_R) = state->velocity.angular.z;

    linearAccel_(0) = state->linear_accel.x;
    linearAccel_(1) = state->linear_accel.y;
    linearAccel_(2) = state->linear_accel.z;
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
    bool valid = false;
    if (type == auv_msgs::Trajectory::BASIC_ABS_XYZ)
        valid = true;
    else if (type == auv_msgs::Trajectory::BASIC_REL_XYZ)
        valid = true;
    return valid;
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
        auv_guidance::Vector13d ref;
        auv_guidance::Vector6d accel;
        ref.setZero();
        accel.setZero();

        double dt = ros::Time::now().toSec() - timeStart_.toSec();

        if (tgenType_ == auv_msgs::Trajectory::BASIC_ABS_XYZ || tgenType_ == auv_msgs::Trajectory::BASIC_ABS_XYZ)
        {
            ref = basicTrajectory_->computeState(dt);
            accel = basicTrajectory_->computeAccel(dt);
            ROS_INFO("Time in Trajectory: %f", dt);
            std::cout << "Reference state: " << std::endl << ref << std::endl; // Debug
            std::cout << "Accel state: " << std::endl << accel << std::endl; // Debug
        }

        if (dt > trajectoryDuration_ && !resultMessageSent_)
        {
            resultMessageSent_ = true;
            auv_msgs::TrajectoryGeneratorResult result;
            result.completed = true;
            tgenActionServer_->setSucceeded(result);
        }

        thrust_ = auvModel_->computeLQRThrust(state_, ref, accel);
        GuidanceController::publishThrustMessage();
    }
}

/**
 * \brief Initialize new trajectory
 */
void GuidanceController::initNewTrajectory()
{
    newTrajectory_ = false;
    timeStart_ = ros::Time::now();

    Eigen::Vector3d zero3d = Eigen::Vector3d::Zero();
    Eigen::Vector3d posIStart = zero3d;
    Eigen::Vector3d velIStart = zero3d;
    Eigen::Vector3d accelIStart = zero3d;
    
    // Inertial position, velocity, and accel expressed in I-frame
    posIStart = state_.segment<3>(acc::STATE_XI);
    velIStart = quaternion_ * state_.segment<3>(acc::STATE_U);
    accelIStart = quaternion_ * linearAccel_;

    startWaypt_ = new auv_guidance::Waypoint(posIStart, velIStart, accelIStart, quaternion_, state_.segment<3>(acc::STATE_P));

    if (tgenType_ == auv_msgs::Trajectory::BASIC_ABS_XYZ || tgenType_ == auv_msgs::Trajectory::BASIC_ABS_XYZ)
    {  
        Eigen::Vector3d posIEnd = zero3d;
        Eigen::Quaterniond quatEnd;

        tf::pointMsgToEigen(desiredTrajectory_.pose.position, posIEnd);
        tf::quaternionMsgToEigen(desiredTrajectory_.pose.orientation, quatEnd);

        if (tgenType_ == auv_msgs::Trajectory::BASIC_REL_XYZ)
            posIEnd = (quaternion_ * posIStart) + posIEnd;

        endWaypt_ = new auv_guidance::Waypoint(posIEnd, zero3d, zero3d, quatEnd, zero3d);
        basicTrajectory_ = new auv_guidance::BasicTrajectory(startWaypt_, endWaypt_, tgenLimits_);
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
