#pragma once

#include <memory>
#include "camera.hpp"

namespace Pegasus {

class PinholeCamera : public Camera {

public:

    using SharedPtr = std::shared_ptr<PinholeCamera>;
    using UniquePtr = std::unique_ptr<PinholeCamera>;
    using WeakPtr = std::weak_ptr<PinholeCamera>;   

    /**
     * @brief Construct an undistorted Pinhole Camera model
     * @param width The width of the camera image frame in pixels
     * @param height The height of the camera image frame in pixels
     * @param fx The focal distance of the camera along the x-axis
     * @param fy The focal distance of the camera along the y-axis
     * @param cx The offset of the focal point in the image frame along the x-axis
     * @param cy The offset of the focal point in the image frame aling the y-axis
     */
    PinholeCamera(double width, double height, double fx, double fy, double cx, double cy); 

    /**
     * @brief Destroy the Pinhole Camera object
     */
    ~PinholeCamera();

    /**
     * @brief Method that takes a 3D point expressed in the Camera coordinate frame,
     * projects it to the image plane (using the intrinsics), and outputs the pixels
     * corresponding to that point in the image plane. (pi function)
     * @param point The 3D point expressed in the Camera coordinate frame (do not confuse with inertial/world frame)
     * @param pixel Eigen::Vector2d The corresponding 2D pixel in the image frame
     */
    virtual void project(const Eigen::Vector3d & point, Eigen::Vector2d & pixel) const override;

    /**
     * @brief Method that takes a 2D pixel in the image frame,
     * and backprojects it to the Camera coordinate frame. Since this problem 
     * is under-defined, we only generate a vector from the camera
     * center to the point (expressed in the Camera frame, and which we don't know exactly
     * where it is, but know its direction from the camera center)
     * @param pixel A 2D vector which encodes the pixel (u=horizontal pixel, v=vertical pixel)
     * @param vector The 3D vector from the center of the camera
     * coordinate frame to the 3D point (that we don't know the Z).
     */
    virtual void backproject(const Eigen::Vector2d & pixel, Eigen::Vector3d & vector) const override;

private:

    /**
     * @brief Focal distance of the camera
     */
    const double fx_, fy_;

    /**
     * @brief Offset of the focal point in the image frame
     */
    const double cx_, cy_;

    /**
     * @brief The instrinsics matrix 
     */
    Eigen::Matrix3d K_;

    /**
     * @brief The inverse of the instrinsics matrix 
     */
    Eigen::Matrix3d K_inv_;
};

}