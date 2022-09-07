#pragma once

#include <memory>
#include <Eigen/Core>
#include "section.hpp"

namespace Pegasus::Paths {

class Circle : public Section {

public:

    using SharedPtr = std::shared_ptr<Circle>;
    using UniquePtr = std::unique_ptr<Circle>;
    using WeakPtr = std::weak_ptr<Circle>;

    /**
     * @brief Constructor for a new Circle path section
     * @param vehicle_speed A shared pointer to a desired vehicle speed
     * @param center A 3D vector with the starting point for the line
     * @param normal The normal vector that defines the plane where the 2D circle will be placed
     * @param radius The radius of the circle in meters (m)
     */
    Circle(const std::shared_ptr<Speed> vehicle_speed, const Eigen::Vector3d & center, const Eigen::Vector3d & normal, const double radius);

    /**
     * @brief Constructor for a new Circle path section. This circle will be inscribed in a xy-plane
     * centered at "center"
     * @param vehicle_speed A shared pointer to a desired vehicle speed
     * @param center A 3D vector with the starting point for the line
     * @param radius The radius of the circle in meters (m)
     */
    Circle(const std::shared_ptr<Speed> vehicle_speed, const Eigen::Vector3d & center, const double radius);

    /**
     * @brief The section parametric equation 
     * @param gamma The path parameter
     * @return An Eigen::Vector3d with the equation of the path with respect to the path parameter gamma
     */
    virtual Eigen::Vector3d pd(double gamma) override;

    /**
     * @brief First derivative of the path section equation with respect to path parameter gamma
     * @param gamma The path parameter
     * @return An Eigen::Vector3d with the first derivative of the path equation with respect to the path parameter
     */
    virtual Eigen::Vector3d d_pd(double gamma) override;

    /**
     * @brief Second derivative of the path section equation with respect to the path parameter gamma
     * @param gamma  The path parameter
     * @return An Eigen::Vector3d with the second derivative of the path equation with respect to the path paramter
     */
    virtual Eigen::Vector3d dd_pd(double gamma) override;

    /** 
     * @brief Override and just returns 0.0
     * @param gamma The path parameter
     * @return A double with the line curvature  = 0
     */
    virtual double curvature(double gamma) override;

private:

    /**
     * @brief The starting point of the line
     */
    Eigen::Vector3d center_;

    /**
     * @brief The normal vector of the plane where the circle will be located. By default
     * the circle will be in the xy-plane located in z=center[2], without any fancy rotation
     * applied to it. If the normal vector has some other value, then a rotation will be applied
     * to the plane
     */
    Eigen::Vector3d normal_{0.0, 0.0, 1.0};

    /**
     * @brief The rotation matrix to apply based on the normal vector to the plane where the circle
     * should be inscribed. By default, this is the identity matrix
     */
    Eigen::Matrix3d rotation_{Eigen::Matrix3d::Identity()};

    /**
     * @brief The radius of the circle
     */
    double radius_;

    /**
     * @brief The curvature of the circle
     */
    double curvature_;

};
}