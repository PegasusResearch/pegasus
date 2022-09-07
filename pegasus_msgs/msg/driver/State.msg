# position and orientation in inertial frame NED
geometry_msgs/PoseStamped pose
# body linear vel (u,v, w) and body angular vel (p,q,r) (f.r.d)
geometry_msgs/TwistStamped body_vel
# roll, pitch and yaw (euler angles in inertial frame NED)
geometry_msgs/Vector3Stamped rpy
# inertial linear velocity (x_dot, y_dot, z_dot)
geometry_msgs/Vector3Stamped inertial_vel