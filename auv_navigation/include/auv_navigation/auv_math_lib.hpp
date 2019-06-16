#ifndef AUV_MATH_LIB
#define AUV_MATH_LIB

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"

using namespace Eigen;

// Useful math tools
namespace AUVMathLib
{
Matrix3f getRotationMat(int axis, float angle);

Matrix3f getEulerRotationMat(const Ref<const Vector3f> &attitude);

MatrixXf sign(const Ref<const MatrixXf>& mat);

int sign(double x);
int sign(float x);
int sign(int x);

Matrix3f skewSym(const Ref<const Vector3f> &v);

float sawtoothWave(float x, float period, float max);
float triangularWave(float x, float period, float max);
float rollYawMap(float x);
float pitchMap(float x);
Vector3f getConstrainedAttitude(const Ref<const Vector3f> attitude);

Vector3f eulerDot2PQR(const Ref<const Vector3f> attitude, const Ref<const Vector3f> eulerDot);
Vector3f pqr2EulerDot(const Ref<const Vector3f> attitude, const Ref<const Vector3f> pqr);

Vector4d quaternion2AngleAxis(const Quaterniond &quaternion);
Quaterniond angleAxis2Quaternion(const Ref<const Vector4d> &angleAxis);
Quaterniond toQuaternion(double yaw, double pitch, double roll);
Vector3d toEulerAngle(const Quaterniond &quaternion);
} // namespace AUVMathLib

#endif