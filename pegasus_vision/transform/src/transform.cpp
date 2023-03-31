#include "transform/transform.hpp"

#include <Eigen/Dense>

namespace Pegasus {

/**
 * @brief Create a transform object that represent a 4x4 homogeneous transformation matrix
 */
Transform::Transform() {

  // Set the rotation to the identity quaternion and the translation to zero
  q_.setIdentity();
  t_.setZero();
}

/**
 * @brief Create a transform object that represent a 4x4 homogeneous transformation matrix
 * @param T The 4x4 homogeneous transformation matrix
 */
Transform::Transform(const Eigen::Matrix4d& T) {
  
  // Set the rotation from the 3x3 rotation matrix 
  q_ = Eigen::Quaterniond(T.block<3,3>(0,0));

  // Set the translation from the 3x1 translation vector
  t_ = T.block<3,1>(0,3);
}

/**
 * @brief Get the translation vector
 * @return The translation vector
 */
const Eigen::Vector3d& Transform::translation(void) const {
  return t_;
}

/**
 * @brief Get the translation vector
 * @return The translation vector
 */
Eigen::Vector3d& Transform::translation(void) {
  return t_;
}

/**
 * @brief Get the rotation quaternion
 * @return The rotation quaternion
 */
const Eigen::Quaterniond& Transform::rotation(void) const {
  return q_;
}

/**
 * @brief Get the rotation quaternion
 * @return The rotation quaternion
 */
Eigen::Quaterniond& Transform::rotation(void) {
  return q_;
}

/**
 * @brief Get the transform expressed as a 4x4 homogeneous matrix
 * @return The 4x4 homogeneous transformation matrix
 */
Eigen::Matrix4d Transform::to_matrix(void) const {

  // Create a 4x4 homogeneous matrix
  Eigen::Matrix4d T = Eigen::Matrix4d::Zero();

  // Set the rotation matrix
  T.block<3,3>(0,0) = q_.toRotationMatrix();

  // Set the translation vector
  T.block<3,1>(0,3) = t_;

  return T;
}

}