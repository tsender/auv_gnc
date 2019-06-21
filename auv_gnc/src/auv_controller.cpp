#include "auv_gnc/auv_controller.hpp"

namespace auv_gnc
{
AUVController::AUVController(ros::NodeHandle nh)
{
    nh_ = nh;

    nh_.param("subscriber_topic", subTopic_, std::string("/auv_gnc/trans_ekf/six_dof"));
    nh_.param("publisher_topic", pubTopic_, std::string("thrust"));

    sixDoFSub_ = nh_.subscribe<auv_msgs::SixDoF>(subTopic_, 1, &TransEKF::sixDofCB, this);
    sixDoFPub_ = nh_.advertise<auv_msgs::SixDoF>(pubTopic_, 1, this);

    TransEKF::initEKF();
}

/**
 * \brief Initialize all EKF matrices
 */
void AUVController::initEKF()
{
    
}

/**
 * \brief Process new six DoF data thru EKF
 */
void AUVController::sixDofCB(const auv_msgs::SixDoF::ConstPtr &state)
{
    
}

} // namespace auv_gnc
