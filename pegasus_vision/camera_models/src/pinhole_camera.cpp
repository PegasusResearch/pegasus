#include <Eigen/LU>
#include "camera_models/pinhole_camera.hpp"

namespace Pegasus {

/**
 * @brief Construct an undistorted Pinhole Camera model
 * @param width The width of the camera image frame in pixels
 * @param height The height of the camera image frame in pixels
 * @param fx The focal distance of the camera along the x-axis
 * @param fy The focal distance of the camera along the y-axis
 * @param cx The offset of the focal point in the image frame along the x-axis
 * @param cy The offset of the focal point in the image frame aling the y-axis
 */
PinholeCamera::PinholeCamera(double width, double height, double fx, double fy, double cx, double cy) : Camera(width, height), fx_(fx), fy_(fy), cx_(cx), cy_(cy) {
        
    // Initiate the instrinsics matrix
    K_ << fx_, 0.0, cx_, 
          0.0, fy_, cy_, 
          0.0, 0.0, 1.0;

    // Initiate the inverse instrinsics matrix
    K_inv_ = K_.inverse();
}

/**
 * @brief Destroy the Pinhole Camera object
 */
PinholeCamera::~PinholeCamera() {
    // Do nothing
}

/**
 * @brief Method that takes a 3D point expressed in the Camera coordinate frame,
 * projects it to the image plane (using the intrinsics), and outputs the pixels
 * corresponding to that point in the image plane. (pi function)
 * @param point The 3D point expressed in the Camera coordinate frame (do not confuse with inertial/world frame)
 * @param pixel The corresponding 2D pixel in the image frame
 * @return Eigen::Vector2d The corresponding 2D pixel in the image frame
 */
void PinholeCamera::project(const Eigen::Vector3d & point, Eigen::Vector2d & pixel) const {
    // Compute the camera frame coordinates (in "homogeneous" coordinates), i.e. lambda * [u, v, 1]^T
    // Since we only want "u" and "v", we normalize the vector at the end and just return "u" and "v"
    Eigen::Vector3d uv = K_ * point;
    pixel << uv(0) / uv(2), uv(1) / uv(2);
}

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
void PinholeCamera::backproject(const Eigen::Vector2d & pixel, Eigen::Vector3d & vector) const {
    vector = K_inv_ * Eigen::Vector3d(pixel(0), pixel(1), 1.0);
}

}