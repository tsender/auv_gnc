#include "auv_gnc/trans_ekf.hpp"

namespace auv_gnc
{
TransEKF::TransEKF(ros::NodeHandle nh)
{
   nh_ = nh;
   nh_.param("subscriber_topic", subTopic_, std::string("input_data"));
   nh_.param("publisher_topic", pubTopic_, std::string("six_dof"));

   nh_.param("position_sensing", posSensing_, std::vector<bool>(0));
   nh_.param("R_pos_diag", RposDiag_, std::vector<double>(0));
   nh_.param("R_vel_diag", RvelDiag_, std::vector<double>(0));
   nh_.param("R_accel_diag", RaccelDiag_, std::vector<double>(0));
   nh_.param("Q_diag", QDiag_, std::vector<double>(0));

   sixDoFSub_ = nh_.subscribe<auv_msgs::SixDoF>(subTopic_, 1, &TransEKF::sixDofCB, this);
   sixDoFPub_ = nh_.advertise<auv_msgs::SixDoF>(pubTopic_, 1, this);

   TransEKF::initEKF();
   ROS_INFO("Trans EKF initialized");
}

/**
 * \brief Initialize all EKF matrices
 */
void TransEKF::initEKF()
{
   dt = 0;
   numPosSensing_ = 0;
   Rvel_.setZero();
   Raccel_.setZero();
   Q_.setZero();
   inertialState_.setZero();
   bodyState_.setZero();
   quaternion_.setIdentity();

   for (int i = 0; i < 3; i++)
      if (posSensing_[i])
         numPosSensing_++;

   Rpos_.resize(numPosSensing_, numPosSensing_);
   Rpos_.setZero();
   Eigen::Vector3i posMask = Eigen::Vector3i::Zero();

   int j = 0;
   for (int i = 0; i < 3; i++)
   {
      if (posSensing_[i])
      {
         posMask(i) = 1;
         Rpos_(j, j) = RposDiag_[i];
         j++; // Eigen is weird. Placing the j++ within the Rpos() line messes things up
      }
      Rvel_(i, i) = RvelDiag_[i];
      Raccel_(i, i) = RaccelDiag_[i];
   }

   for (int i = 0; i < 9; i++)
      Q_(i, i) = QDiag_[i];

   transEKF_ = new auv_navigation::TranslationEKF(posMask, Rpos_, Rvel_, Raccel_, Q_);
   init_ = false;
}

/**
 * \brief Process new six DoF data thru EKF
 */
void TransEKF::sixDofCB(const auv_msgs::SixDoF::ConstPtr &raw)
{
   auv_navigation::Vector9d newData;
   newData.setZero();

   // Read new data
   ros::Time timeNew = raw->header.stamp;
   newData(0) = raw->pose.position.x;
   newData(1) = raw->pose.position.y;
   newData(2) = raw->pose.position.z;
   newData(3) = raw->velocity.linear.x;
   newData(4) = raw->velocity.linear.y;
   newData(5) = raw->velocity.linear.z;
   newData(6) = raw->linear_accel.x;
   newData(7) = raw->linear_accel.y;
   newData(8) = raw->linear_accel.z;
   auv_core::eigen_ros::quaternionMsgToEigen(raw->pose.orientation, quaternion_);

   // Create sensor mask
   Eigen::Vector3i sensorMask = Eigen::Vector3i::Zero();

   // Create data matrix and express all measurements in the I-frame
   Eigen::Matrix3d dataMat = Eigen::Matrix3d::Zero();
   dataMat.col(0) = newData.segment<3>(STATE_POS);
   dataMat.col(1) = quaternion_ * newData.segment<3>(STATE_VEL);
   dataMat.col(2) = quaternion_ * newData.segment<3>(STATE_ACCEL);

   if (!init_)
   {
      transEKF_->init(newData);
      init_ = true;
   }
   else
   {
      // Check for which sensor provided new data
      if (!lastMeasurement_.segment<3>(STATE_POS).isApprox(newData.segment<3>(STATE_POS)))
         sensorMask(0) = 1;

      if (!lastMeasurement_.segment<3>(STATE_VEL).isApprox(newData.segment<3>(STATE_VEL)))
         sensorMask(1) = 1;

      if (!lastMeasurement_.segment<3>(STATE_ACCEL).isApprox(newData.segment<3>(STATE_ACCEL)))
         sensorMask(2) = 1;

      if (sensorMask.sum() > 0)
      {
         dt = timeNew.toSec() - timeLast_.toSec();
         inertialState_ = transEKF_->update(dt, sensorMask, dataMat);

         // Express linear velocity and accel in B-frame
         bodyState_ = inertialState_;
         bodyState_.segment<3>(STATE_VEL) = quaternion_.conjugate() * inertialState_.segment<3>(STATE_VEL);
         bodyState_.segment<3>(STATE_ACCEL) = quaternion_.conjugate() * inertialState_.segment<3>(STATE_ACCEL);

         // Create SixDoF message and publish
         auv_msgs::SixDoF filtered;
         filtered.header.stamp = timeNew;
         filtered.header.frame_id = std::string("/auv_gnc/trans_ekf/");

         auv_core::eigen_ros::pointEigenToMsg(inertialState_.segment<3>(STATE_POS), filtered.pose.position);
         filtered.pose.orientation = raw->pose.orientation;
         auv_core::eigen_ros::vectorEigenToMsg(bodyState_.segment<3>(STATE_VEL), filtered.velocity.linear);
         filtered.velocity.angular = raw->velocity.angular;
         auv_core::eigen_ros::vectorEigenToMsg(bodyState_.segment<3>(STATE_ACCEL), filtered.linear_accel);

         sixDoFPub_.publish(filtered);
      }
   }

   lastMeasurement_ = newData;
   timeLast_ = timeNew;
}

} // namespace auv_gnc
