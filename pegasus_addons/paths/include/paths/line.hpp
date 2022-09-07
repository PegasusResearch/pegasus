#pragma once

#include <memory>
#include <Eigen/Core>
#include "section.hpp"

namespace Pegasus::Paths {

class Line : public Section {

public:

    using SharedPtr = std::shared_ptr<Line>;
    using UniquePtr = std::unique_ptr<Line>;
    using WeakPtr = std::weak_ptr<Line>;

    /**
     * @brief Constructor for a new Line path section
     * @param vehicle_speed A shared pointer to a desired vehicle speed
     * @param start A 3D vector with the starting point for the line
     * @param end A 3D vector with the ending point for the line
     */
    Line(const std::shared_ptr<Speed> vehicle_speed, const Eigen::Vector3d & start, const Eigen::Vector3d & end) : 
        Section(vehicle_speed, "line", 0.0, 1.0), start_(start), end_(end) {
            slope_ = end_ - start_;
        }

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
    Eigen::Vector3d start_;

    /**
     * @brief The ending point of the line
     */
    Eigen::Vector3d end_;

    /**
     * @brief The slope of the line
     */
    Eigen::Vector3d slope_;

    /**
     * @brief The constant eigen vector for the second derivative of the line
     */
    Eigen::Vector3d dd_pd_{0.0, 0.0, 0.0};
};
}