#pragma once

#include <memory>
#include <Eigen/Dense>

namespace Pegasus {

/**
 * @brief Class to represent a camera
*/
class Camera {

public:

    /**
     * @brief An alias for a shared pointer of a Camera
     */
    using SharedPtr = std::shared_ptr<Camera>;

    /**
     * @brief An alias for a unique pointer of a Camera
     */
    using UniquePtr = std::unique_ptr<Camera>;

    /**
     * @brief An alias for a weak pointer of a Camera
     */
    using WeakPtr = std::weak_ptr<Camera>;

    void project_points(
        const std::vector<Eigen::Vector3d>& points, 
        std::vector<Eigen::Vector2d>& projected_points) const;

protected:

    int image_width_{0};
    int image_height_{0};
    std::string camera_type_{""};

};
}