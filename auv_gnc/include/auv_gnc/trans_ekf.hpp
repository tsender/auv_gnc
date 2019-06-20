#ifndef AUV_GNC
#define AUV_GNC

#include "auv_navigation/translation_ekf.hpp"
#include "auv_msgs/SixDoF.h"
#include <ros/ros.h>
#include "math.h"
#include <vector>

namespace auv_gnc
{

class TransEKF
{
private:
  auv_navigation::TranslationEKF *transEKF_;
  auv_navigation::Vector9d inertialState_;
  auv_navigation::Vector9d bodyState_;
  auv_navigation::Vector9d lastMeasurement_;
  Eigen::Quaterniond quaternion_;

  Eigen::MatrixXd Rpos_;
  Eigen::Matrix3d Rvel_, Raccel_;
  auv_navigation::Matrix9d Q_;
  std::vector<bool> posSensing_;
  std::vector<double> RposDiag_, RvelDiag_, RaccelDiag_, QDiag_;
  int numPosSensing_;
  bool init_;
  double dt;
  ros::Time timeLast_;

  ros::NodeHandle nh_;
  ros::Subscriber sixDoFSub_;
  ros::Publisher sixDoFPub_;
  std::string subTopic_, pubTopic_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const int STATE_POS = 0;
  static const int STATE_VEL = 3;
  static const int STATE_ACCEL = 6;
  
  TransEKF(ros::NodeHandle nh);
  void initEKF();
  void sixDofCB(const auv_msgs::SixDoF::ConstPtr &raw);
};
} // namespace auv_gnc

#endif