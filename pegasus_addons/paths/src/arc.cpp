#include "paths/arc.hpp"

namespace Pegasus::Paths {

/**
 * @brief Construct a new Arc:: Arc object
 * @param vehicle_speed 
 * @param start 
 * @param end 
 */
Arc::Arc(const std::shared_ptr<Speed> vehicle_speed, const Eigen::Vector3d & start, const Eigen::Vector3d & center, const Eigen::Vector3d & end, const bool clockwise_direction) : Section(vehicle_speed, 0.0, 1.0), start_(start), center_(center), end_(end), clockwise_direction_(clockwise_direction) {

    // Compute the arc radius
    // TODO

    // Compute the angle of the starting point in the circle
    init_angle_ = 

}

/**
 * @brief The section parametric equation 
 * @param gamma The path parameter
 * @return An Eigen::Vector3d with the equation of the path with respect to the path parameter gamma
 */
Eigen::Vector3d Arc::pd(double gamma) {
    
    // Bound the parametric value
    double t = limit_gamma(gamma);

    // Compute the angle of the arc corresponding to the initial point in 2D space

}

/**
 * @brief First derivative of the path section equation with respect to path parameter gamma
 * @param gamma The path parameter
 * @return An Eigen::Vector3d with the first derivative of the path equation with respect to the path parameter
 */
Eigen::Vector3d Arc::d_pd(double gamma) {
    
    // Bound the parametric value
    double t = limit_gamma(gamma);
}

/**
 * @brief Second derivative of the path section equation with respect to the path parameter gamma
 * @param gamma  The path parameter
 * @return An Eigen::Vector3d with the second derivative of the path equation with respect to the path paramter
 */
Eigen::Vector3d Arc::dd_pd(double gamma) {
    
    // Bound the parametric value
    double t = limit_gamma(gamma);
}

/** 
 * @brief Override and just returns 0.0
 * @param gamma The path parameter
 * @return A double with the line curvature  = 0
 */
double Arc::curvature(double gamma) {
    
    // Bound the parametric value
    double t = limit_gamma(gamma);
}

}