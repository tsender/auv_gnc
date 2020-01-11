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

Eigen::Quaterniond relativeQuat(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2);

Eigen::Matrix3d getRotationMat(int axis, double angle);

Eigen::Matrix3d getEulerRotationMat(const Eigen::Ref<const Eigen::Vector3d> &rpy);

Eigen::Matrix3d skewSym(const Eigen::Ref<const Eigen::Vector3d> &v);

float sawtoothWave(float x, float period, float max);

float triangularWave(float x, float period, float max);

float mapRollYaw(float x);

float mapPitch(float x);

Eigen::Vector3d getConstrainedRPY(const Eigen::Ref<const Eigen::Vector3d> attitude);

Eigen::Vector3d pqr2RPYDot(const Eigen::Ref<const Eigen::Vector3d> rpy, const Eigen::Ref<const Eigen::Vector3d> pqr);

Eigen::Vector3d rpyDot2PQR(const Eigen::Ref<const Eigen::Vector3d> rpy, const Eigen::Ref<const Eigen::Vector3d> rpyDot);
} // namespace rot3d
} // namespace auv_core

#endif