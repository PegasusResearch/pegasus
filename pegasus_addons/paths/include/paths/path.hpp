#pragma once

#include <vector>
#include <optional>
#include "section.hpp"

namespace Pegasus::Paths {

class Path {

public:

    using SharedPtr = std::shared_ptr<Path>;
    using UniquePtr = std::unique_ptr<Path>;
    using WeakPtr = std::weak_ptr<Path>;

    /**
     * @brief Construct a new Path object
     */
    Path() = default;

    /**
     * @brief Destroy the Path object
     */
    ~Path();

    /**
     * @brief Add a path section shared pointer to the end of the path
     * @param section A shared pointer to a generic path section
     * @return bool returns true if the section is successfully added or false if not
     */
    virtual bool push_back(const Section::SharedPtr section);

    /**
     * @brief Get a reference to a section, given the parametric value
     * @param gamma The parametric value
     * @return Section::SharedPtr 
     */
    virtual Section::SharedPtr get_section(double gamma);

    /**
     * @brief Get the index of a given section for a parametric value inside the 
     * sections vector
     * @param gamma The parametric value
     * @return unsigned int The index inside the section vector
     */
    virtual std::optional<unsigned int> get_section_index(double gamma);

    /**
     * @brief Get the string which describes the type of path section that is setored for a particular
     * parametric value of the full path
     * @param gamma The parametric value
     * @return std::string A description of the path section type
     */
    virtual std::optional<std::string> get_section_type(double gamma);

    /**
     * @brief Clear the path (removes all segments if any were added)
     */
    virtual void clear();

    /**
     * @brief Check whether the path is empty or contains any section
     * @return bool True if the path is empty
     */
    virtual inline bool empty() { return sections_.empty(); }

    /**
     * @brief Check the number of sections that the path contains
     * @return unsigned int The number of sections inside the path
     */
    virtual inline unsigned int size() { return sections_.size(); };

    /**
     * @brief The section parametric equation 
     * @param gamma The path parameter
     * @return std::optional<Eigen::Vector3d> The equation of the path evaluated at the path parameter gamma
     */
    virtual std::optional<Eigen::Vector3d> pd(double gamma) const;

    /**
     * @brief First derivative of the path section equation with respect to path parameter gamma
     * @param gamma The path parameter
     * @return std::optional<Eigen::Vector3d> The second derivative of the path equation evaluated at the path parameter gamma
     */
    virtual std::optional<Eigen::Vector3d> d_pd(double gamma) const;

    /**
     * @brief Second derivative of the path section equation with respect to the path parameter gamma
     * @param gamma  The path parameter
     * @return std::optional<double> The second derivative of the path equation evaluated at the path parameter gamma
     */
    virtual std::optional<Eigen::Vector3d> dd_pd(double gamma) const;

    /**
     * @brief Default method for computing the curvature. The default implementation
     * implements the general formula to compute the curvature based on the derivative
     * equations of the path
     * @param gamma The path parameter
     * @return std::optional<double> The path curvature 
     */
    virtual std::optional<double> curvature(double gamma) const;

    /**
     * @brief Default method for computing the torsion. The default implementation
     * implements the general formula to compute the curvature based on the derivative
     * equations of the path
     * @param gamma The path parameter
     * @return std::optional<double> The path torsion
     */
    virtual std::optional<double> torsion(double gamma) const;

    /**
     * @brief Default method for computing the tangent angle to the path 
     * @param gamma The path parameter
     * @return std::optional<double> The angle of the tangent to the path expressed in radians
     */
    virtual std::optional<double> tangent_angle(double gamma) const;

    /**
     * @brief Default method for computing the norm of the derivative 
     * @param gamma  The path parameter
     * @return std::optional<double> The norm of the derivative of the path position pd
     */
    virtual std::optional<double> derivative_norm(double gamma) const;

    /**
     * @brief Default method for getting the desired vehicle speed for a particular 
     * location in the path section (in m/s)
     * @param gamma The path parameter
     * @return std::optional<double> The desired vehicle speed (in m/s)
     */
    virtual std::optional<double> vehicle_speed(double gamma) const;

    /**
     * @brief Default method for getting the desired speed for the evolution
     * of the parametric value
     * @param The path parameter
     * @return std::optional<double> The desired speed progression of the parametric variable
     */
    virtual std::optional<double> vd(double gamma) const;

    /**
     * @brief Default method for getting the desired acceleration for the evolution
     * of the parametric value
     * @param gamma The path parameter
     * @return std::optional<double> The speed progression of the parametric variable
     */
    virtual std::optional<double> d_vd(double gamma) const;

protected:

    /**
     * @brief Method that outputs a bound parametric value between minimum and the maximum
     * parametric value that a path can achieve based on the maximum number of sections it contains
     * at any given time.
     * @param gamma The path parameter
     * @return double The bounded path parameter
     */
    virtual double bound_gamma(double gamma);

    /**
     * @brief Vector of parametric curves parameterized between 0 and 1
     */
    std::vector<Section::SharedPtr> sections_;
};

}