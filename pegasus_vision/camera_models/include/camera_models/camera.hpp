#pragma once 

#include <Eigen/Core>

namespace Pegasus {

/**
 * @brief Abstract class that will be used as a base for all cameras (Pinhole, Fish-eye, atan, etc.)
 */
class Camera {

public:    

    using SharedPtr = std::shared_ptr<Camera>;
    using UniquePtr = std::unique_ptr<Camera>;
    using WeakPtr = std::weak_ptr<Camera>;

    /**
     * @brief Destroy the Abstract Camera object
     */
    virtual ~Camera() {}

    /**
     * @brief Method that takes a 3D point expressed in the Camera coordinate frame,
     * projects it to the image plane (using the intrinsics), and outputs the pixels
     * corresponding to that point in the image plane
     * @param x The X coordinate of the point in the Camera coordinate frame (do not confuse with inertial/world frame)
     * @param y The Y coordinate of the point in the Camera coordinate frame (do not confuse with the inertial/world frame)
     * @param z The Z coordinate of the point in the Camera coordinate frame (do not confuse with the inertial/world frame)
     * @return Eigen::Vector2d The corresponding 2D pixel in the image frame
     */
    virtual Eigen::Vector2d project(const double & x, const double & y, const double & z) = 0;

    /**
     * @brief Method that takes a 3D point expressed in the Camera coordinate frame,
     * projects it to the image plane (using the intrinsics), and outputs the pixels
     * corresponding to that point in the image plane
     * @param point The 3D point expressed in the Camera coordinate frame (do not confuse with inertial/world frame)
     * @return Eigen::Vector2d The corresponding 2D pixel in the image frame
     */
    virtual Eigen::Vector2d project(const Eigen::Vector3d & point) = 0;

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
    virtual Eigen::Vector3d backproject(const double & u, const double & v) = 0;

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
    virtual Eigen::Vector3d backproject(const Eigen::Vector2d & pixel) = 0;

    /**
     * @brief Returns the width (in pixels) of the images produced by this camera
     */
    inline unsigned int width() { return width_; }

    /**
     * @brief Returns the height (in pixels) of the images produced by this camera
     */
    inline unsigned int height() { return height_; }

    /**
     * @brief Method that given a 2D vector representing a pixel coordinate, checks
     * if it is possible for that pixel coordinate to be in an image frame acquired by this camera
     * @param pixel The 2D coordinates of the pixel
     * @return bool A boolean whether the pixel lies in the image plane of this camera or not
     */
    inline bool is_pixel_in_frame(const Eigen::Vector2d & pixel) { return is_pixel_in_frame(pixel[0], pixel[1]); }

    /**
     * @brief Method that given a 2D vector representing a pixel coordinate, checks
     * if it is possible for that pixel coordinate to be in an image frame acquired by this camera
     * @param u The "x" coordinate of the pixel in the image frame
     * @param v The "y" coordinate of the pixel in the image frame
     * @return bool A boolean whether the pixel lies in the image plane of this camera or not
     */
    inline bool is_pixel_in_frame(const double & u, const double & v) {
        return (u >= 0.0 && u < width_ && v >= 0.0 && v < height_) ? true : false;
    }


protected:

    /**
     * @brief Constructor for a general abstract Camera object
     * @param width The width of the camera image frame in pixels
     * @param height The height of the camera image frame in pixels
     */
    Camera(const int &  width, const int & height) : width_(width), height_(height) {}

    /**
     * @brief The dimensions of the image frame that this camera generates (in pixels)
     */
    unsigned int width_, height_;

};

}