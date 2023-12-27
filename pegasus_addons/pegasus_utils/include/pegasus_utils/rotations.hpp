/**
 * Authors:
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 *      Andre Potes (andre.potes@tecnico.ulisboa.pt)
 * Maintained by: Marcelo Fialho Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Last Update: 14/12/2021
 * License: MIT
 * File: rotations.hpp 
 * Brief: Defines all functions related to angle wrapping, rotation matrices, 
 * euler angles, convertion to quaternions, etc.
 */
#pragma once

#include <Eigen/Core>
#include <cmath>

/**
 * @brief Function to convert from quaternion to (roll, pitch and yaw), according to Z-Y-X convention
 * This function is from: https://github.com/mavlink/mavros/issues/444
 * and the logic is also available at: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * @param q An eigen quaternion
 * Authors:
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 *      Andre Potes (andre.potes@tecnico.ulisboa.pt)
 * Maintained by: Marcelo Fialho Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Last Update: 14/12/2021
 * License: MIT
 * File: rotations.hpp 
 * Brief: Defines all functions related to angle wrapping, rotation matrices, 
 * euler angles, convertion to quaternions, etc.
 */
#pragma once

#include <Eigen/Core>
#include <cmath>

namespace Pegasus {

namespace Rotations {

/**
 * @brief Function to convert from quaternion to (roll, pitch and yaw), according to Z-Y-X convention
 * This function is from: https://github.com/mavlink/mavros/issues/444
 * and the logic is also available at: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * @param q An eigen quaternion
 * @return A Vector<T, 3> with the [roll, pitch, yaw] obtained according to Z-Y-X convention
 */
template <typename T>
inline Eigen::Matrix<T, 3, 1> quaternion_to_euler(const Eigen::Quaternion<T> &q) {
    /* NOTE: The Eigen standard way of doing it is not used because for the order YPR the output range would be:
    [Eigen EulerAngles implementation] yaw, pitch, roll in the ranges [0:pi]x[-pi:pi]x[-pi:pi] */

    Eigen::Matrix<T, 3, 1> rpy;

    /* Compute roll */
    rpy.x() = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y()*q.y()));
    T sin_pitch = 2 * (q.w()*q.y() - q.z()*q.x());
    sin_pitch = sin_pitch >  1 ?  1 : sin_pitch;
    sin_pitch = sin_pitch < -1 ? -1 : sin_pitch;

    /* Compute pitch */
    rpy.y() = std::asin(sin_pitch);

    /* Compute yaw */
    rpy.z() = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

    return rpy;
}

/**
 * @brief Converts a vector of euler angles according to Z-Y-X convention
 * into a quaternion
 * @param v An eigen vector of either floats or doubles [roll, pitch, yaw]
 * @return An Eigen Quaternion
 */
template <typename T>
inline Eigen::Quaternion<T> euler_to_quaternion(const Eigen::Matrix<T, 3, 1> &v) {
    
    // Create the Eigen quaternion
    Eigen::Quaternion<T> orientation;

    // Obtain the orientation according to Z-Y-X convention
    orientation = Eigen::AngleAxis<T>(v.z(), Eigen::Matrix<T, 3, 1>::UnitZ()) *
            Eigen::AngleAxis<T>(v.y(), Eigen::Matrix<T, 3, 1>::UnitY()) *
            Eigen::AngleAxis<T>(v.x(), Eigen::Matrix<T, 3, 1>::UnitX());

    return orientation;
}

/**
 * @brief Gets the yaw angle from a quaternion (assumed a Z-Y-X rotation)
 * NOTE: this function is based on: 
 * https://github.com/mavlink/mavros/blob/ros2/mavros/src/lib/ftf_quaternion_utils.cpp
 * which in turn has the theory explained in:
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * @param q A eigen quaternion
 * @return The yaw angle in radians (assumed a Z-Y-X rotation)
 */
template <typename T>
inline T yaw_from_quaternion(const Eigen::Quaternion<T> &q) {
    return std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
}

/**
 * @brief Wrap angle between [0, 2PI] 
 * 
 * @param angle angle in radians
 * @return The wraped angle
 */
template <typename T>
inline T wrapTo2pi(T angle) {

    double wrapped_angle = std::fmod(angle, 2 * M_PI);

    if(wrapped_angle < 0) 
        wrapped_angle += 2 * M_PI;
    return wrapped_angle;
}

/**
 * @brief Wrap angle between [-PI, PI] 
 * 
 * @param angle angle in radians
 * @return The wraped angle
 */
template <typename T>
inline T wrapTopi(T angle) {

    double wrapped_angle = std::fmod(angle + M_PI, 2 * M_PI);

    if (wrapped_angle < 0)
        wrapped_angle += 2 * M_PI;

    return wrapped_angle - M_PI;
}

/**
 * @brief Convert an angle in radian to degrees
 * 
 * @param angle in radians
 * @return angle in degrees
 */
template <typename T>
inline T rad_to_deg(T angle) {
    return angle * 180 / M_PI;
}

/**
 * @brief Convert an angle in degrees to radians
 * 
 * @param angle in degrees
 * @return angle in radians
 */
template <typename T>
inline T deg_to_rad(T angle) {
    return angle * M_PI / 180;
}

/**
 * @brief Method to calculate the diference between angles correctly even if they wrap between -pi and pi
 * 
 * @param a angle 1 in radians 
 * @param b angle 2 in radians
 * @return The minimum difference between the two angles 
 */
template <typename T>
inline T angleDiff(T a, T b) {
    double aux = std::fmod(a - b + M_PI, 2 * M_PI);
    if (aux < 0) aux += (2 * M_PI);
    aux = aux - M_PI;
    return aux;
}


/**
 * @brief Compute the 3x3 skew-symmetric matrix from a vector 3x1
 * @param v A vector with 3 elements
 * @return A 3x3 skew-symmetric matrix
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> computeSkewSymmetric3(const Eigen::Matrix<T, 3, 1> &v) {

    Eigen::Matrix<T, 3, 3> skew_symmetric;
    skew_symmetric <<    0, -v(2),  v(1),
                      v(2),     0, -v(0),
                     -v(1),  v(0),     0;

    return skew_symmetric;
}

/** 
 * @brief Compute the Vee Map of a 3x3 matrix
 * @param m A 3x3 matrix
 * @return A 3x1 vector
*/
template <typename T>
inline Eigen::Matrix<T,3,1> computeVeeMap(const Eigen::Matrix<T,3,3> &m) {
    Eigen::Matrix<T,3,1> v;
    v << m(2,1), m(0,2), m(1,0);
    return v;
}

/**
 * @brief Compute the 2x2 skew-symmetric matrix from a constant (int, float or double)
 * @param v A constant
 * @return A 2x2 skew-symmetric matrix
 */
template <typename T>
inline Eigen::Matrix<T, 2, 2> computeSkewSymmetric2(T c) {

    Eigen::Matrix<T, 2, 2> skew_symmetric;
    skew_symmetric << 0, -c,
                      c,  0;

    return skew_symmetric;
}

/**
 * @brief Compute the rotation matrix that converts angular velocities expressed in the body frame
 * to angular velocities expressed in the inertial frame (according to Z-Y-X convention) - makes use of small angle approximation
 * @param v A vector with 3 elements (roll, pitch, yaw)
 * @return A 3x3 rotation matrix
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> rotationAngularBodyToInertial(const Eigen::Matrix<T, 3, 1> &v) {
    Eigen::Matrix<T, 3, 3> transformation_matrix;
    transformation_matrix << 1, sin(v(0)) * tan(v(1)), cos(v(0)) * tan(v(1)),
                             0, cos(v(0)), -sin(v(0)),
                             0, sin(v(0)) / cos(v(1)), cos(v(0)) / cos(v(1));
    return transformation_matrix;
}

/**
 * @brief Method that returns a rotation matrix from body frame to inertial frame, assuming a Z-Y-X convention
 * @param v A vector with euler angles (roll, pith, yaw) according to Z-Y-X convention
 * @return A 3x3 rotation matrix
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> rotationBodyToInertial(const Eigen::Matrix<T, 3, 1> &v) {
    
    // Create a quaternion
    Eigen::Matrix<T, 3, 3> m;

    // Obtain the orientation according to Z-Y-X convention
    m = (Eigen::AngleAxis<T>(v.z(), Eigen::Matrix<T, 3, 1>::UnitZ()) *
         Eigen::AngleAxis<T>(v.y(), Eigen::Matrix<T, 3, 1>::UnitY()) *
         Eigen::AngleAxis<T>(v.x(), Eigen::Matrix<T, 3, 1>::UnitX())).toRotationMatrix();
    
    return m;
}
}}