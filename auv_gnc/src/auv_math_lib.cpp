#include "auv_gnc/auv_math_lib.h"

namespace AUV_GNC
{
namespace AUVMathLib
{
// Return rotation matrix about a single axis
Matrix3f GetAxisRotation(int axis, float angle)
{
    Matrix3f R = Matrix3f::Zero();
    RowVector3f row1, row2, row3;

    if (axis == 1) // X Axis
    {
        row1 << 1, 0, 0;
        row2 << 0, cos(angle), sin(angle);
        row3 << 0, -sin(angle), cos(angle);
    }
    else if (axis == 2) // Y Axis
    {
        row1 << cos(angle), 0, -sin(angle);
        row2 << 0, 1, 0;
        row3 << sin(angle), 0, cos(angle);
    }
    else if (axis == 3) // Z Axis
    {
        row1 << cos(angle), sin(angle), 0;
        row2 << -sin(angle), cos(angle), 0;
        row3 << 0, 0, 1;
    }
    else // Return Identity if axis not valid
    {
        row1 << 1, 0, 0;
        row2 << 0, 1, 0;
        row3 << 0, 0, 1;
    }

    R << row1, row2, row3;
    return R;
}

// Get rotation matrix from world-frame to B-frame using Euler Angles (yaw, pitch, roll)
// Ex. Vb = R * Vw, Vb = vector in B-frame coordinates, Vw = vector in world-frame coordinates
// Parameters:
//      attitude = Eigen::Vector3f of yaw, pitch, and roll (in this order)
Matrix3f GetEulerRotMat(const Ref<const Vector3f> &attitude)
{
    Matrix3f R = Matrix3f::Identity();

    int axis = 1;
    for (int i = 2; i >= 0; i--)
        R = R * AUVMathLib::GetAxisRotation(axis++, attitude(i)); // R1(phi) * R2(theta) * R3(psi)

    return R;
}

// Compute vehicle acceleration expressed in the body-frame
// Parameters:
//   velBF = vel expressed in body-frame
//   angVelBF = ang. vel expressed in the body-frame
MatrixXf SgnMat(const Ref<const MatrixXf>& mat)
{
    MatrixXf sgnMat = mat;

    for (int i = 0; i < mat.rows(); i++)
    {
        for (int j = 0; j < mat.cols(); j++)
        {
            if (mat(i, j) > 0)
                sgnMat(i, j) = 1;
            else if (mat(i, j) < 0)
                sgnMat(i, j) = -1;
            else
                sgnMat(i, j) = 0;
        }
    }
    return sgnMat;
}

//template <typename T>
int Sgn(double &x){
    if (x > 0)
        return 1;
    else 
        return -1;
}

int Sgn(float &x){
    if (x > 0)
        return 1;
    else 
        return -1;
}

int Sgn(int &x){
    if (x > 0)
        return 1;
    else 
        return -1;
}

Matrix3f SkewSym(const Ref<const Vector3f> &v)
{
    Matrix3f skew = Matrix3f::Zero();
    RowVector3f row1, row2, row3;

    row1 << 0, -v(2), v(1);
    row2 << v(2), 0, -v(0);
    row3 << -v(1), v(0), 0;

    skew << row1, row2, row3;
    return skew;
}

// Constrain 'x' to conform to the sawtooth profile
float SawtoothWave(float x, float period, float max)
{
    return max * 2 * (x/period - floor(0.5 + x/period));
}


// Constrain 'x' to conform to the triangulat profile
float TriangularWave(float x, float period, float max)
{
    float f = floor(0.5 + 2*x/period);
    return max * 2 * (2*x/period - f) * pow(-1,f);
}

// Returns value within the bounds of roll/yaw values: [-180, +180] deg
// Follows sawtooth profile
float RollYawMap(float x)
{
    return SawtoothWave(x, 360, 180);
}

// Returns value within the bounds of pitch values: [-90, +90] deg
// Follows triangular profile
float PitchMap(float x)
{
    return TriangularWave(x, 360, 90);
}
} // namespace AUVMathLib
} // namespace AUV_GNC