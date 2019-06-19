#ifndef AUV_GNC
#define AUV_GNC

#include "auv_navigation/translation_ekf.hpp"
#include "auv_msgs/SixDoF.h"
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include "math.h"
#include <vector>

namespace auv_gnc
{

class TransEKF
{
private:
  auv_navigation::TranslationEKF *transEKF_;
  auv_navigation::Vector9f inertialState_;
  auv_navigation::Vector9f bodyState_;
  auv_navigation::Vector9f lastMeasurement_;

  Eigen::MatrixXf Rpos_;
  Eigen::Matrix3f Rvel_, Raccel_;
  auv_navigation::Matrix9f Q_;
  bool posSensing_[3];
  int numPosSensing_;

  YAML::Node config_;
  std::string configFile_;

  ros::NodeHandle nh_;
  ros::Subscriber sub6dof_;
  ros::Publisher pub6DofInertial_, pub6DofBody_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TransEKF(ros::NodeHandle nh);

  template <typename T>
  void loadParam(std::string param, T &var);
  void configureEKF();
  //init(auv_navigation::Vector9f);
};
} // namespace auv_gnc

#endif