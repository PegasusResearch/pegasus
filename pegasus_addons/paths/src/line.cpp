#include "paths/line.hpp"

namespace Pegasus::Paths {

/**
 * @brief The section parametric equation 
 * @param gamma The path parameter
 * @return An Eigen::Vector3d with the equation of the path with respect to the path parameter gamma
 */
Eigen::Vector3d Line::pd(double gamma) {
    
    // Bound the parametric value
    double t = limit_gamma(gamma);
    return slope_ * t + start_;
}

/**
 * @brief First derivative of the path section equation with respect to path parameter gamma
 * @param gamma The path parameter
 * @return An Eigen::Vector3d with the first derivative of the path equation with respect to the path parameter
 */
Eigen::Vector3d Line::d_pd(double gamma) {
    (void) gamma;
    return slope_;
}

/**
 * @brief Second derivative of the path section equation with respect to the path parameter gamma
 * @param gamma  The path parameter
 * @return An Eigen::Vector3d with the second derivative of the path equation with respect to the path paramter
 */
Eigen::Vector3d Line::dd_pd(double gamma) {
    (void) gamma;
    return dd_pd_;
}

/** 
 * @brief Override and just returns 0.0
 * @param gamma The path parameter
 * @return A double with the line curvature  = 0
 */
double Line::curvature(double gamma) {
    (void) gamma;
    return 0.0;
}

}