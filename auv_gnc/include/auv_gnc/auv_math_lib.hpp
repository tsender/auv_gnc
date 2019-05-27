#ifndef AUV_MATH_LIB
#define AUV_MATH_LIB

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"

using namespace Eigen;

namespace AUV_GNC
{
// Useful math tools
namespace AUVMathLib
{
Matrix3f getAxisRotation(int axis, float angle);

Matrix3f getEulerRotMat(const Ref<const Vector3f> &attitude);

MatrixXf sign(const Ref<const MatrixXf>& mat);

int sign(double x);
int sign(float x);
int sign(int x);

Matrix3f skewSym(const Ref<const Vector3f> &v);

float sawtoothWave(float x, float period, float max);
float triangularWave(float x, float period, float max);
float rollYawMap(float x);
float pitchMap(float x);
} // namespace AUVMathLib
} // namespace AUV_GNC

#endif