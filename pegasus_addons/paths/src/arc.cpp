#include "paths/arc.hpp"

namespace Pegasus::Paths {

/**
 * @brief Construct a new Arc section object
 * @param vehicle_speed The vehicle speed object
 * @param start The starting point where the arc is defined
 * @param center The center point of the arc
 * @param The normal vector that defines the plane where the 2D arc will be placed
 * @param clockwise_direction Whether the arc should be performed in clock or anti-clockwise direction
 */
Arc::Arc(const std::shared_ptr<Speed> vehicle_speed, const Eigen::Vector3d & start, const Eigen::Vector3d & center, const Eigen::Vector3d & normal, const bool clockwise_direction) : Arc(vehicle_speed, start, center, clockwise_direction) {

    // ------------------------
    // Initialize the rotation matrix with the rotation 
    // information encoded in the normal vector
    // -----------------------
    
    // Step 1 - Check that the normal vector != [0, 0, 1] and [0,0, -1] otherwize we would not need to rotate
    // and the rotation matrix would be hill-posed using the following method
    Eigen::Vector3d base_normal(0.0, 0.0, 1.0);
    Eigen::Vector3d absolute_normal = normal_.array().abs();

    // Because we are using doubles, we must check if the vectors are not approximatelly [0, 0, 1] 
    if ((absolute_normal - base_normal).norm() > 0.0001) {

        // Step 2 - Now that we know that the vector is valid, then compute the rotation matrix
        Eigen::Vector3d u3 = normal_.normalized();
        Eigen::Vector3d u1 = (u3.cross3(base_normal)).normalized();
        Eigen::Vector3d u2 = (u3.cross3(u1)).normalized();

        // Step 3 - assign the normalized vectors to the columns of the rotation matrix
        rotation_.col(0) = u1;
        rotation_.col(1) = u2;
        rotation_.col(2) = u3;
    }

}

/**
 * @brief Construct a new Arc section object
 * @param vehicle_speed The vehicle speed object
 * @param start The starting point where the arc is defined
 * @param center The center point of the arc
 * @param clockwise_direction Whether the arc should be performed in clock or anti-clockwise direction
 */
Arc::Arc(const std::shared_ptr<Speed> vehicle_speed, const Eigen::Vector3d & start, const Eigen::Vector3d & center, const bool clockwise_direction) : Section(vehicle_speed, 0.0, 1.0), start_(start), center_(center) {

    // Set the clockwise direction variable to be -1 or 1
    clockwise_direction_ = (clockwise_direction) ? 1.0 : -1.0;

    // Compute the arc radius
    radius_ = (center - start).norm();

    // Compute the angle of the starting point in the circle
    init_angle_ = std::atan2(start[1] - center[1], start[0] - center[0]);

    // Compute the curvature
    curvature_ = clockwise_direction_ * 1.0 / radius_;
}

/**
 * @brief The section parametric equation 
 * @param gamma The path parameter
 * @return An Eigen::Vector3d with the equation of the path with respect to the path parameter gamma
 */
Eigen::Vector3d Arc::pd(double gamma) {
    
    // Bound the parametric value
    double t = limit_gamma(gamma);

    // Compute the angle of the arc corresponding to the point in 2D space, according to the parametric value
    double curr_angle = init_angle_ - clockwise_direction_ * t * M_PI;

    // Compute the location of the 2D arc in a plane centered around [x y, 0.0]
    Eigen::Vector3d pd;
    pd[0] = radius_ * cos(curr_angle);
    pd[1] = radius_ * sin(curr_angle);
    pd[2] = 0.0;

    // If the "normal_" vector is different than [0.0, 0.0, 1.0], then 
    // rotate the plane where the circle is located. Otherwise, we are just multiplying by the identity matrix
    pd = rotation_ * pd;

    // Add theoffset to the circle after the rotation, otherwise the offset would also get rotated
    return pd + center_;
}

/**
 * @brief First derivative of the path section equation with respect to path parameter gamma
 * @param gamma The path parameter
 * @return An Eigen::Vector3d with the first derivative of the path equation with respect to the path parameter
 */
Eigen::Vector3d Arc::d_pd(double gamma) {
    
    // Bound the parametric value
    double t = limit_gamma(gamma);

    // Compute the angle of the arc corresponding to the point in 2D space, according to the parametric value
    double curr_angle = init_angle_ - clockwise_direction_ * t * M_PI;

    // Compute the location of the 2D arc in a plane centered around [x y, 0.0]
    Eigen::Vector3d d_pd;
    d_pd[0] = -radius_ * sin(curr_angle) * (-clockwise_direction_ * M_PI);
    d_pd[1] = radius_ * cos(curr_angle) * (-clockwise_direction_ * M_PI);
    d_pd[2] = 0.0;

    // If the "normal_" vector is different than [0.0, 0.0, 1.0], then 
    // rotate the plane where the circle is located. Otherwise, we are just multiplying by the identity matrix
    d_pd = rotation_ * d_pd;

    // Add theoffset to the circle after the rotation, otherwise the offset would also get rotated
    return d_pd + center_;
}

/**
 * @brief Second derivative of the path section equation with respect to the path parameter gamma
 * @param gamma  The path parameter
 * @return An Eigen::Vector3d with the second derivative of the path equation with respect to the path paramter
 */
Eigen::Vector3d Arc::dd_pd(double gamma) {
    
    // Bound the parametric value
    double t = limit_gamma(gamma);

    // Compute the angle of the arc corresponding to the point in 2D space, according to the parametric value
    double curr_angle = init_angle_ - clockwise_direction_ * t * M_PI;

    // Compute the location of the 2D arc in a plane centered around [x y, 0.0]
    Eigen::Vector3d dd_pd;
    dd_pd[0] = -radius_ * cos(curr_angle) * std::pow(-clockwise_direction_ * M_PI, 2);
    dd_pd[1] = -radius_ * sin(curr_angle) * std::pow(-clockwise_direction_ * M_PI, 2);
    dd_pd[2] = 0.0;

    // If the "normal_" vector is different than [0.0, 0.0, 1.0], then 
    // rotate the plane where the circle is located. Otherwise, we are just multiplying by the identity matrix
    dd_pd = rotation_ * dd_pd;

    // Add theoffset to the circle after the rotation, otherwise the offset would also get rotated
    return dd_pd + center_;
}

/** 
 * @brief Override and just returns a constant
 * @param gamma The path parameter
 * @return A double with the arc curvature
 */
double Arc::curvature(double gamma) {
    return curvature_;
}

}