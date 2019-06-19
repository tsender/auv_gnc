std_msgs/Header Header
geometry_msgs/Pose pose # Position [m] expressed in inertial frame, quaternion from inertial to body frame
geometry_msgs/Twist velocity # Both linear [m/s] and angular [rad/s] velocities expressed in the body frame
geometry_msgs/Vector3 linear_accel # Linear acceleration [m/s^2]