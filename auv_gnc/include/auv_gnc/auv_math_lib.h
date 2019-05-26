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
Matrix3f GetAxisRotation(int axis, float angle);

Matrix3f GetEulerRotMat(const Ref<const Vector3f> &attitude);

MatrixXf SgnMat(const Ref<const MatrixXf>& mat);

int Sgn(double &x);
int Sgn(float &x);
int Sgn(int &x);

Matrix3f SkewSym(const Ref<const Vector3f> &v);

float SawtoothWave(float x, float period, float max);
float TriangularWave(float x, float period, float max);
float RollYawMap(float x);
float PitchMap(float x);
} // namespace AUVMathLib
} // namespace AUV_GNC

#endif