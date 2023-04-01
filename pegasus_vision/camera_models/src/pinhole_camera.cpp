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
 * @brief Method that takes a 2D pixel in the image frame,
 * and backprojects it to the Camera coordinate frame. Since this problem 
 * is under-defined, we only generate the normalized vector from the camera
 * center to the point (expressed in the Camera frame, and which we don't know exactly
 * where it is, but know its direction from the camera center)
 * @param pixel A 2D vector which encodes the pixel (u=horizontal pixel, v=vertical pixel)
 * @return Eigen::Vector3d The normalized 3D vector from the center of the camera
 * coordinate frame to the 3D point (that we don't know the Z). This vector is returned
 * normalized (as a unit vector)
 */
Eigen::Vector3d PinholeCamera::backproject(const Eigen::Vector2d & pixels) {
    return backproject(pixels(0), pixels(1));
}

/**
 * @brief Method that takes a 2D pixel in the image frame,
 * and backprojects it to the Camera coordinate frame. Since this problem 
 * is under-defined, we only generate the normalized vector from the camera
 * center to the point (expressed in the Camera frame, and which we don't know exactly
 * where it is, but know its direction from the camera center)
 * @param u The horizontal pixel
 * @param v The vertical pixel
 * @return Eigen::Vector3d The normalized 3D vector from the center of the camera
 * coordinate frame to the 3D point (that we don't know the Z). This vector is returned
 * normalized (as a unit vector)
 */
Eigen::Vector3d PinholeCamera::backproject(const double & u, const double & v) {
    // Declare a vector that will store the normalized backprojection vector (aka direction) 
    // from the camera center to the point in the camera 3D frame of reference. 
    // Note: this is a vector and not a point because we cannot know the Z.
    return (K_inv_ * Eigen::Vector3d(u, v, 1.0)).normalized();
}

/**
 * @brief Method that takes a 3D point expressed in the Camera coordinate frame,
 * projects it to the image plane (using the intrinsics), and outputs the pixels
 * corresponding to that point in the image plane
 * @param x The X coordinate of the point in the Camera coordinate frame (do not confuse with inertial/world frame)
 * @param y The Y coordinate of the point in the Camera coordinate frame (do not confuse with the inertial/world frame)
 * @param z The Z coordinate of the point in the Camera coordinate frame (do not confuse with the inertial/world frame)
 * @return Eigen::Vector2d The corresponding 2D pixel in the image frame
 */
Eigen::Vector2d PinholeCamera::project(const double & x, const double & y, const double & z) {
    return project(Eigen::Vector3d(x, y, z));
}

/**
 * @brief Method that takes a 3D point expressed in the Camera coordinate frame,
 * projects it to the image plane (using the intrinsics), and outputs the pixels
 * corresponding to that point in the image plane
 * @param point The 3D point expressed in the Camera coordinate frame (do not confuse with inertial/world frame)
 * @return Eigen::Vector2d The corresponding 2D pixel in the image frame
 */
Eigen::Vector2d PinholeCamera::project(const Eigen::Vector3d & point) {
    // Compute the camera frame coordinates (in "homogeneous" coordinates), i.e. lambda * [u, v, 1]^T
    // Since we only want "u" and "v", we normalize the vector at the end and just return "u" and "v"
    Eigen::Vector3d uv = K_ * point;
    return Eigen::Vector2d(uv(0) / uv(2), uv(1) / uv(2));
}


}