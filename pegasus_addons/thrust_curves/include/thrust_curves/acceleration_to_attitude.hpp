#pragma once

#include <Eigen/Core>

/**
 * @brief Method that given a desired acceleration to apply to the multirotor,
 * its mass and desired yaw angle (in radian), computes the desired attitude to apply to the vehicle.
 * It returns an Eigen::Vector4d object which contains [roll, pitch, yaw, thrust]
 * with each element expressed in the following units [rad, rad, rad, Newton] respectively.
 * 
 * @param acceleration The desired acceleration to apply to the vehicle in m/s^2
 * @param mass The mass of the vehicle in Kg
 * @param yaw The desired yaw angle of the vehicle in radians
 * @return Eigen::Vector4d object which contains [roll, pitch, yaw, thrust]
 * with each element expressed in the following units [rad, rad, rad, Newton] respectively.
 */
inline Eigen::Vector4d get_attitude_thrust_from_acceleration(const Eigen::Vector3d & acceleration, double mass, double yaw) {

    Eigen::Matrix3d RzT;
    Eigen::Vector3d r3d, u_bar;
    Eigen::Vector4d attitude_thrust;

    /* Compute the normalized thrust and r3d vector */
    double T = mass * acceleration.norm();
    r3d = - acceleration / acceleration.norm();

    /* Compute the rotation matrix about the Z-axis */
    RzT << cos(yaw), sin(yaw), 0.0,
          -sin(yaw), cos(yaw), 0.0,
                0.0,      0.0, 1.0;

    /* Compute the normalized rotation */
    u_bar = RzT * r3d;

    /* Compute the actual attitude and setup the desired thrust to apply to the vehicle */
    attitude_thrust << asin(-u_bar[0]), atan2(u_bar[0], u_bar[2]), yaw, T;
    return attitude_thrust;
}