#pragma once

#include <memory>
#include <Eigen/Dense>

namespace Pegasus {

/**
 * @brief Class to represent a 3D transform
*/
class Transform {

public:

    /**
     * @brief An alias for a shared pointer of a Transform
     */
    using SharedPtr = std::shared_ptr<Transform>;

    /**
     * @brief An alias for a unique pointer of a Transform
     */
    using UniquePtr = std::unique_ptr<Transform>;

    /**
     * @brief An alias for a weak pointer of a Transform
     */
    using WeakPtr = std::weak_ptr<Transform>;

    /**
     * @brief Create a transform object that represent a 4x4 homogeneous transformation matrix
     */
    Transform();
    
    /**
     * @brief Create a transform object that represent a 4x4 homogeneous transformation matrix
     * @param T The 4x4 homogeneous transformation matrix
    */
    Transform(const Eigen::Matrix4d& T);

    /**
     * @brief Get the translation vector
     * @return The translation vector
    */
    const Eigen::Vector3d& translation(void) const;

    /**
     * @brief Get the translation vector
     * @return The translation vector
    */
    Eigen::Vector3d& translation(void);

    /**
     * @brief Get the rotation quaternion
     * @return The rotation quaternion
    */
    const Eigen::Quaterniond& rotation(void) const;

    /**
     * @brief Get the rotation quaternion
     * @return The rotation quaternion
    */
    Eigen::Quaterniond& rotation(void);
    
    /**
     * @brief Get the transform expressed as a 4x4 homogeneous matrix
     * @return The 4x4 homogeneous transformation matrix
    */
    Eigen::Matrix4d to_matrix(void) const;

protected:

    /**
     * @brief The quaternion representation of the rotation
    */
    Eigen::Quaterniond q_;

    /**
     * @brief The translation vector
    */
    Eigen::Vector3d t_;

};

}