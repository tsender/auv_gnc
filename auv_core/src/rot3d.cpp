#include "auv_core/rot3d.hpp"

namespace auv_core
{
namespace rot3d
{
/**
 * @param quaternion Quaternion representation of attitude
 * /brief Convert quaternion to its angle-axis representation. Caution, if the quaternion's scalar component is too close to 1, then the entire angleAxis vector returned is the zero vector.
 */
Eigen::Vector4d quat2AngleAxis(const Eigen::Quaterniond &quaternion)
{
   Eigen::Vector4d angleAxis;
   angleAxis.setZero();
   Eigen::Quaterniond q = quaternion.normalized();
   double val = 1 - q.w() * q.w();

   angleAxis(0) = 2 * acos(q.w()); // Angle [rad]
   if (val != 0)                   // If val == 0, all axis components are zero
   {
      angleAxis(1) = q.x() / sqrt(val); // Axis, x-component
      angleAxis(2) = q.y() / sqrt(val); // Axis, y-component
      angleAxis(3) = q.z() / sqrt(val); // Axis, z-component
      angleAxis.tail<3>().normalize();
   }

   return angleAxis;
}

/**
 * @param angleAxis Angle-axis representation for a quaternion
 * \brief Convert angle-axis representation into quaternion
 */
Eigen::Quaterniond angleAxis2Quat(const Eigen::Ref<const Eigen::Vector4d> &angleAxis)
{
   Eigen::Quaterniond q;
   double angle = angleAxis(0);
   Eigen::Vector3d axis = angleAxis.tail<3>().normalized();
   q.w() = cos(angle / 2.0);
   q.x() = axis(0) * sin(angle / 2.0);
   q.y() = axis(1) * sin(angle / 2.0);
   q.z() = axis(2) * sin(angle / 2.0);

   return q.normalized();
}

/**
 * @param roll Roll in [rad]
 * @param pitch Pitch in [rad]
 * @param yaw Yaw in [rad]
 * \brief Convert roll, pitch, yaw (derived using the 3-2-1 rotation sequence) to a quaternion
 */
Eigen::Quaterniond rpy2Quat(double roll, double pitch, double yaw)
{
   // Abbreviations for the various angular functions
   double cy = cos(yaw * 0.5);
   double sy = sin(yaw * 0.5);
   double cp = cos(pitch * 0.5);
   double sp = sin(pitch * 0.5);
   double cr = cos(roll * 0.5);
   double sr = sin(roll * 0.5);

   Eigen::Quaterniond q;
   q.w() = cy * cp * cr + sy * sp * sr;
   q.x() = cy * cp * sr - sy * sp * cr;
   q.y() = sy * cp * sr + cy * sp * cr;
   q.z() = sy * cp * cr - cy * sp * sr;
   return q.normalized();
}

/**
 * @param quaternion Input quaternion
 * \brief Convert quaternion to (roll, pitch, yaw) in [rad] using the 3-2-1 rotation sequence
 */
Eigen::Vector3d quat2RPY(const Eigen::Quaterniond &quaternion)
{
   Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
   Eigen::Quaterniond q = quaternion.normalized();

   // Roll (x-axis rotation)
   double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
   double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
   rpy(0) = atan2(sinr_cosp, cosr_cosp);

   // Pitch (y-axis rotation)
   double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
   if (fabs(sinp) >= 1)
      rpy(1) = copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
   else
      rpy(1) = asin(sinp);

   // Yaw (z-axis rotation)
   double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
   double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
   rpy(2) = atan2(siny_cosp, cosy_cosp);

   return rpy;
}

/**
 * @param axis Indicates which axis to obtain rotation matrix from, where 0 = X, 1 = Y, 2 = Z
 * @param angle Angle of rotation in [rad]
 * \brief Get rotation matrix about specified axis
 */
Eigen::Matrix3d getRotationMat(int axis, double angle)
{
   Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
   Eigen::RowVector3d row1, row2, row3;

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

/**
 * @param rpy Attitude vector of (roll, pitch, yaw) in [rad] using the 3-2-1 rotation sequence
 * \brief Get rotation matrix from world-frame to body-frame. Ex. Vb = R * Vw, where Vb is expressed in the B-frame ad Vw is expressed in the W-frame. 
 */
Eigen::Matrix3d getEulerRotationMat(const Eigen::Ref<const Eigen::Vector3d> &rpy)
{
   Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

   int axis = 1;
   for (int i = 0; i < 3; i++)
      R = R * rot3d::getRotationMat(axis++, rpy(i)); // R1(phi) * R2(theta) * R3(psi)

   return R;
}

/**
 * @param v 3D input vector
 * \brief Get the skew-symmetric matrix from input vector
 */
Eigen::Matrix3d skewSym(const Eigen::Ref<const Eigen::Vector3d> &v)
{
   Eigen::Matrix3d skew = Eigen::Matrix3d::Zero();
   Eigen::RowVector3d row1, row2, row3;

   row1 << 0, -v(2), v(1);
   row2 << v(2), 0, -v(0);
   row3 << -v(1), v(0), 0;

   skew << row1, row2, row3;
   return skew;
}

/**
 * @param x Value to be mapped via sawtooth wave
 * @param period Period of sawtooth wave in [rad]
 * @param max Maximum value in sawtooth wave
 * \brief Map input 'x' via sawtooth wave profile
 */
float sawtoothWave(float x, float period, float max)
{
   return max * 2 * (x / period - floor(0.5 + x / period));
}

/**
 * @param x Value to be mapped via triangular wave
 * @param period Period of triangular wave in [rad]
 * @param max Maximum value in triangular wave
 * \brief Map input 'x' via triangular wave profile
 */
float triangularWave(float x, float period, float max)
{
   float f = floor(0.5 + 2 * x / period);
   return max * 2 * (2 * x / period - f) * pow(-1, f);
}

/**
 * @param x Roll/Yaw value in [rad]
 * \brief Maps input to within roll/yaw bounds = [-pi, +pi] [rad] using the sawtooth profile.
 */
float mapRollYaw(float x)
{
   return rot3d::sawtoothWave(x, 2 * M_PI, M_PI);
}

/**
 * @param x Pitch value in [rad]
 * \brief Maps input to within pitch bounds = [-pi/2, +pi/2] [rad] using the triangular profile.
 */
float mapPitch(float x)
{
   return rot3d::triangularWave(x, 2 * M_PI, M_PI / 2);
}

/**
 * @param rpy Attitude vector of (roll, pitch, yaw) in [rad] using the 3-2-1 rotation sequence
 * \brief Constrains roll/yaw to [-pi,pi] in [rad] and pitch to [-pi/2,+pi/2] in [rad]
 */
Eigen::Vector3d getConstrainedRPY(const Eigen::Ref<const Eigen::Vector3d> rpy)
{
   Eigen::Vector3d constrainedRPY = Eigen::Vector3d::Zero();
   constrainedRPY(0) = rot3d::mapRollYaw(rpy(0));
   constrainedRPY(1) = rot3d::mapPitch(rpy(1));
   constrainedRPY(2) = rot3d::mapRollYaw(rpy(2));
   return constrainedRPY;
}

/**
 * @param rpy Attitude vector of (roll, pitch, yaw) in [rad] using the 3-2-1 rotation sequence 
 * @param pqr Angular velocity about body-frame x,y,z axes in [rad/s] or [deg/s]
 * \brief Returns the instantaneous time-derivative of rpy in the same units as pqr
 */
Eigen::Vector3d pqr2RPYDot(const Eigen::Ref<const Eigen::Vector3d> rpy, const Eigen::Ref<const Eigen::Vector3d> pqr)
{
   float cosr = cos(rpy(0));
   float sinr = sin(rpy(0));
   float cosp = cos(rpy(1));
   float tanp = tan(rpy(1));

   Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
   Eigen::RowVector3d row1, row2, row3;

   row1 << 1, sinr * tanp, cosr * tanp;
   row2 << 0, cosr, -sinr;
   row3 << 0, sinr / cosp, cosr / cosp;

   mat << row1, row2, row3;
   return mat * pqr; // rpy dot
}

/**
 * @param rpy Attitude vector of (roll, pitch, yaw) in [rad] using the 3-2-1 rotation sequence 
 * @param rpyDot Time-derivative of rpy [rad/s] or [deg/s]
 * \brief Returns angular velocity about body-frame x,y,z axes with same units as rpyDot
 */
Eigen::Vector3d rpyDot2PQR(const Eigen::Ref<const Eigen::Vector3d> rpy, const Eigen::Ref<const Eigen::Vector3d> rpyDot)
{
   float cosr = cos(rpy(0));
   float sinr = sin(rpy(0));
   float cosp = cos(rpy(1));
   float sinp = sin(rpy(1));

   Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
   Eigen::RowVector3d row1, row2, row3;

   row1 << 1, 0, -sinp;
   row2 << 0, cosr, sinr * cosp;
   row3 << 0, -sinr, cosr * cosp;

   mat << row1, row2, row3;
   return mat * rpyDot; // pqr
}
} // namespace rot3d
} // namespace auv_core