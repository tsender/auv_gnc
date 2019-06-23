#ifndef MATH_LIB
#define MATH_LIB

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"

// Useful math tools
namespace auv_core
{
namespace math_lib
{
Eigen::Matrix3f getRotationMat(int axis, float angle);

Eigen::Matrix3f getEulerRotationMat(const Eigen::Ref<const Eigen::Vector3f> &attitude);

Eigen::MatrixXf sign(const Eigen::Ref<const Eigen::MatrixXf> &mat);

int sign(double x);
int sign(float x);
int sign(int x);

Eigen::Matrix3f skewSym(const Eigen::Ref<const Eigen::Vector3f> &v);

float sawtoothWave(float x, float period, float max);
float triangularWave(float x, float period, float max);
float rollYawMap(float x);
float pitchMap(float x);
Eigen::Vector3f getConstrainedAttitude(const Eigen::Ref<const Eigen::Vector3f> attitude);

Eigen::Vector3f eulerDot2PQR(const Eigen::Ref<const Eigen::Vector3f> attitude, const Eigen::Ref<const Eigen::Vector3f> eulerDot);
Eigen::Vector3f pqr2EulerDot(const Eigen::Ref<const Eigen::Vector3f> attitude, const Eigen::Ref<const Eigen::Vector3f> pqr);

Eigen::Vector4d quaternion2AngleAxis(const Eigen::Quaterniond &quaternion);
Eigen::Quaterniond angleAxis2Quaternion(const Eigen::Ref<const Eigen::Vector4d> &angleAxis);
Eigen::Quaterniond toQuaternion(double yaw, double pitch, double roll);
Eigen::Vector3d toEulerAngle(const Eigen::Quaterniond &quaternion);
} // namespace math_lib
} // namespace auv_core

#endif