#include "auv_gnc/trans_ekf.hpp"

namespace auv_gnc
{
TransEKF::TransEKF(ros::NodeHandle nh)
{
    nh_ = nh;
    TransEKF::loadParam<std::string>("config_file", configFile_);
    config_ = YAML::LoadFile(configFile_);
    
    TransEKF::configureEKF();
}

template <typename T>
void TransEKF::loadParam(std::string param, T &var)
{
    try
    {
        if (!nh_.getParam(param, var))
        {
            throw 0;
        }
    }
    catch (int e)
    {
        std::string ns = nh_.getNamespace();
        ROS_ERROR("TransEKF Namespace: %s", ns.c_str());
        ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
        ros::shutdown();
    }
}

void TransEKF::configureEKF()
{
    Rvel_.setZero();
    Raccel_.setZero();
    numPosSensing_ = 0;
    
    for (int i = 0; i < 3; i++)
    {
        posSensing_[i] = config_["ekf"]["position_sensing"][i].as<bool>();
        numPosSensing_ += (int)posSensing_[i];
    }

    Eigen::MatrixXf Rpos(numPosSensing_, numPosSensing_);
    Rpos.setZero();
    
    int j = 0;
    for (int i = 0; i < 3; i++)
    {
        if (posSensing_[i])
        {
            Rpos(j, j) = config_["ekf"]["R_pos"][i].as<double>();
            j++;
        }
    }

    std::cout << Rpos << std::endl;

}

} // namespace auv_gnc
