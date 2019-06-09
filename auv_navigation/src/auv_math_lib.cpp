#include "auv_navigation/auv_math_lib.hpp"

namespace AUVMathLib
{
// Return rotation matrix about a single axis
// Angle is in [rad]
Matrix3f getRotationMat(int axis, float angle)
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

// Get rotation matrix from world-frame to B-frame using Euler Angles (roll, pitch, yaw)
// Ex. Vb = R * Vw, Vb = vector in B-frame coordinates, Vw = vector in world-frame coordinates
// Parameters:
//      attitude = Eigen::Vector3f of roll, pitch, and yaw [rad] (in this order)
Matrix3f getEulerRotationMat(const Ref<const Vector3f> &attitude)
{
    Matrix3f R = Matrix3f::Identity();

    int axis = 1;
    for (int i = 0; i < 3; i++)
        R = R * AUVMathLib::getRotationMat(axis++, attitude(i)); // R1(phi) * R2(theta) * R3(psi)

    return R;
}

// Compute vehicle acceleration expressed in the body-frame
// Parameters:
//   velBF = vel expressed in body-frame
//   angVelBF = ang. vel expressed in the body-frame
MatrixXf sign(const Ref<const MatrixXf> &mat)
{
    MatrixXf signMat(mat.rows(), mat.cols());
    signMat.setZero();

    for (int i = 0; i < mat.rows(); i++)
    {
        for (int j = 0; j < mat.cols(); j++)
        {
            if (mat(i, j) > 0)
                signMat(i, j) = 1;
            else if (mat(i, j) < 0)
                signMat(i, j) = -1;
        }
    }
    return signMat;
}

int sign(double x)
{
    if (x > 0)
        return 1;
    else if (x < 0)
        return -1;
    return 0;
}

int sign(float x)
{
    if (x > 0)
        return 1;
    else if (x < 0)
        return -1;
    return 0;
}

int sign(int x)
{
    if (x > 0)
        return 1;
    else if (x < 0)
        return -1;
    return 0;
}

Matrix3f skewSym(const Ref<const Vector3f> &v)
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
float sawtoothWave(float x, float period, float max)
{
    return max * 2 * (x / period - floor(0.5 + x / period));
}

// Constrain 'x' to conform to the triangulat profile
float triangularWave(float x, float period, float max)
{
    float f = floor(0.5 + 2 * x / period);
    return max * 2 * (2 * x / period - f) * pow(-1, f);
}

// Returns value within the bounds of roll/yaw values: [-180, +180] deg = [-pi, +pi] rad
// Follows sawtooth profile
// x is in [rad]
float rollYawMap(float x)
{
    return AUVMathLib::sawtoothWave(x, 2*M_PI, M_PI);
}

// Returns value within the bounds of pitch values: [-90, +90] deg = [-pi/2, +pi/2] rad
// Follows triangular profile
// x is in [rad]
float pitchMap(float x)
{
    return AUVMathLib::triangularWave(x, 2*M_PI, M_PI/2);
}

// Constrains roll/yaw to [-180,+180] deg and pitch to [-90,+90] deg
// Attitude in (roll, pitch, yaw) [rad]
Vector3f getConstrainedAttitude(const Ref<const Vector3f> attitude)
{
    Vector3f constrainedAttitude = Vector3f::Zero();
    constrainedAttitude(0) = AUVMathLib::rollYawMap(attitude(0));
    constrainedAttitude(1) = AUVMathLib::pitchMap(attitude(1));
    constrainedAttitude(2) = AUVMathLib::rollYawMap(attitude(2));
    return constrainedAttitude;
}

// Compute body-rates (PQR) from Euler angle rates (rollDot, pitchDot, yawDot) [same unit as input]
// Attitude in (roll, pitch, yaw) [rad]
// eulerDot in [rad/s]
Vector3f eulerDot2PQR(const Ref<const Vector3f> attitude, const Ref<const Vector3f> eulerDot)
{
    float phi = attitude(0);
    float theta = attitude(1);
    float psi = attitude(2);

    Matrix3f mat = Matrix3f::Zero();
    RowVector3f row1, row2, row3;

    row1 << 1, 0, -sin(theta);
    row2 << 0 , cos(phi), sin(phi) * cos(theta);
    row3 << 0, -sin(phi), cos(phi) * cos(theta);

    mat << row1, row2, row3;
    Vector3f pqr = mat * eulerDot;
    return pqr;
}

// Compute Euler angle rates (rollDot, pitchDot, yawDot) from body-rates (PQR) [same unit as input]
// Attitude in [rad]
// pqr in [rad/s]
Vector3f pqr2EulerDot(const Ref<const Vector3f> attitude, const Ref<const Vector3f> pqr)
{
    float phi = attitude(0);
    float theta = attitude(1);
    float psi = attitude(2);

    Matrix3f mat = Matrix3f::Zero();
    RowVector3f row1, row2, row3;

    row1 << 1, sin(phi) * tan(theta), cos(phi) * tan(theta);
    row2 << 0 , cos(phi), -sin(phi);
    row3 << 0 , sin(phi) / cos(theta), cos(phi) / cos(theta);

    mat << row1, row2, row3;
    Vector3f eulerDot = mat * pqr;
    return eulerDot;
}

Vector4d quaternion2AngleAxis(const Quaterniond &quaternion)
{
    Vector4d angleAxis;
    angleAxis.setZero();
    Quaterniond q = quaternion.normalized();
    double i = 1 - q.w() * q.w();

    angleAxis(0) = 2 * acos(q.w()); // Angle [rad]
    angleAxis(1) = q.x() / sqrt(i); // Axis, x-component
    angleAxis(2) = q.y() / sqrt(i); // Axis, y-component
    angleAxis(3) = q.z() / sqrt(i); // Axis, z-component

    return angleAxis;
}

Quaterniond toQuaternion(double yaw, double pitch, double roll)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaterniond q;
    q.w() = cy * cp * cr + sy * sp * sr;
    q.x() = cy * cp * sr - sy * sp * cr;
    q.y() = sy * cp * sr + cy * sp * cr;
    q.z() = sy * cp * cr - cy * sp * sr;
    return q.normalized();
}

Vector3d toEulerAngle(const Quaterniond &quaternion)
{
	Vector3d rpy = Vector3d::Zero();
    Quaterniond q = quaternion.normalized();

    // roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	rpy(0) = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		rpy(1) = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		rpy(1) = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
	rpy(2) = atan2(siny_cosp, cosy_cosp);

    return rpy;
}

} // namespace AUVMathLib