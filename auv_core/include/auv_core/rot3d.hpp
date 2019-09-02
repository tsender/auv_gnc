#ifndef ROT3D
#define ROT3D

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"

// Useful math tools
namespace auv_core
{
namespace rot3d
{
Eigen::Vector4d quat2AngleAxis(const Eigen::Quaterniond &quaternion);

Eigen::Quaterniond angleAxis2Quat(const Eigen::Ref<const Eigen::Vector4d> &angleAxis);

Eigen::Quaterniond rpy2Quat(double roll, double pitch, double yaw);

Eigen::Vector3d quat2RPY(const Eigen::Quaterniond &quaternion);

Eigen::Matrix3f getRotationMat(int axis, float angle);

Eigen::Matrix3f getEulerRotationMat(const Eigen::Ref<const Eigen::Vector3f> &rpy);

Eigen::Matrix3f skewSym(const Eigen::Ref<const Eigen::Vector3f> &v);

float sawtoothWave(float x, float period, float max);

float triangularWave(float x, float period, float max);

float mapRollYaw(float x);

float mapPitch(float x);

Eigen::Vector3f getConstrainedRPY(const Eigen::Ref<const Eigen::Vector3f> attitude);

Eigen::Vector3f pqr2RPYDot(const Eigen::Ref<const Eigen::Vector3f> rpy, const Eigen::Ref<const Eigen::Vector3f> pqr);

Eigen::Vector3f rpyDot2PQR(const Eigen::Ref<const Eigen::Vector3f> rpy, const Eigen::Ref<const Eigen::Vector3f> rpyDot);
} // namespace rot3d
} // namespace auv_core

#endif