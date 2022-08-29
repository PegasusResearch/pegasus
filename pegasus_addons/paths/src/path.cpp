#include "paths/path.hpp"

namespace Pegasus::Paths {
/**
 * @brief Construct a new Path object
 */
Path::Path() {

}

/**
 * @brief Destroy the Path object
 */
~Path::Path() {

}

/**
 * @brief Add a path section shared pointer to the end of the path
 * @param section A shared pointer to a generic path section
 * @return bool returns true if the section is successfully added or false if not
 */
bool Path::push_back(const Section::SharedPtr section) {
    sections_.push_back(section);
}

/**
 * @brief Get a reference to a section, given the parametric value
 * @param gamma The parametric value
 * @return Section::SharedPtr 
 */
Section::SharedPtr Path::get_section(double gamma) {
    // Check if there is a section for the given gamma. If so return a shared pointer to it, otherwise return null
    return (std::optional<unsigned int> index = get_section_index(gamma)) ? sections_[index] : nullptr;
}

/**
 * @brief Get the index of a given section for a parametric value inside the 
 * sections vector
 * @param gamma The parametric value
 * @return unsigned int The index inside the section vector
 */
std::optional<unsigned int> Path::get_section_index(double gamma) {
    // Given that each section is normalized between 0 and 1, then if we floor a given gamma, let's say gamma = 0.5
    // then we know that it belongs to section of index=0.
    // If we got gamma = 5.9, then it belonged to the section of index=5 and we could peak its properties
    // if we query section of index=5 with gamma=0.9
    return (!empty()) ? std::optional<unsigned int>(std::floor(gamma)) : std::nullopt;
}

/**
 * @brief Get the string which describes the type of path section that is setored for a particular
 * parametric value of the full path
 * @param gamma The parametric value
 * @return std::string A description of the path section type
 */
std::optional<std::string> Path::get_section_type(double gamma) {
    // If there is a section for the given gamma, then access it and get its type (can be a line, circle, arc, lemniscate, ...)
    return (std::optional<unsigned int> index = get_section_index(gamma)) ? sections_[index]->get_section_type() : std::nullopt;
}

/**
 * @brief Clear the path (removes all segments if any were added)
 */
void Path::clear() {
    // Empty the sections vector
    sections_.clear();
}

/**
 * @brief The section parametric equation 
 * @param gamma The path parameter
 * @return std::optional<Eigen::Vector3d> The equation of the path evaluated at the path parameter gamma
 */
std::optional<Eigen::Vector3d> Path::pd(double gamma) const {

    // Make the gamma vary between 0 and 1 for a given path section
    double normalized_gamma = gamma - std::floor(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    return (std::optional<unsigned int> index = get_section_index(gamma)) ? sections_[index]->pd(normalized_gamma) : std::nullopt;
}

/**
 * @brief First derivative of the path section equation with respect to path parameter gamma
 * @param gamma The path parameter
 * @return std::optional<Eigen::Vector3d> The second derivative of the path equation evaluated at the path parameter gamma
 */
std::optional<Eigen::Vector3d> Path::d_pd(double gamma) const {
    // Make the gamma vary between 0 and 1 for a given path section
    double normalized_gamma = gamma - std::floor(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    return (std::optional<unsigned int> index = get_section_index(gamma)) ? sections_[index]->d_pd(normalized_gamma) : std::nullopt;

}

/**
 * @brief Second derivative of the path section equation with respect to the path parameter gamma
 * @param gamma  The path parameter
 * @return std::optional<double> The second derivative of the path equation evaluated at the path parameter gamma
 */
std::optional<Eigen::Vector3d> Path::dd_pd(double gamma) const {
    // Make the gamma vary between 0 and 1 for a given path section
    double normalized_gamma = gamma - std::floor(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    return (std::optional<unsigned int> index = get_section_index(gamma)) ? sections_[index]->dd_pd(normalized_gamma) : std::nullopt;

}

/**
 * @brief Default method for computing the curvature. The default implementation
 * implements the general formula to compute the curvature based on the derivative
 * equations of the path
 * @param gamma The path parameter
 * @return std::optional<double> The path curvature 
 */
std::optional<double> Path::curvature(double gamma) const {
    // Make the gamma vary between 0 and 1 for a given path section
    double normalized_gamma = gamma - std::floor(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    return (std::optional<unsigned int> index = get_section_index(gamma)) ? sections_[index]->curvature(normalized_gamma) : std::nullopt;

}

/**
 * @brief Default method for computing the torsion. The default implementation
 * implements the general formula to compute the curvature based on the derivative
 * equations of the path
 * @param gamma The path parameter
 * @return std::optional<double> The path torsion
 */
std::optional<double> Path::torsion(double gamma) const {
    // Make the gamma vary between 0 and 1 for a given path section
    double normalized_gamma = gamma - std::floor(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    return (std::optional<unsigned int> index = get_section_index(gamma)) ? sections_[index]->torsion(normalized_gamma) : std::nullopt;
}

/**
 * @brief Default method for computing the tangent angle to the path 
 * @param gamma The path parameter
 * @return std::optional<double> The angle of the tangent to the path expressed in radians
 */
std::optional<double> Path::tangent_angle(double gamma) const {
    // Make the gamma vary between 0 and 1 for a given path section
    double normalized_gamma = gamma - std::floor(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    return (std::optional<unsigned int> index = get_section_index(gamma)) ? sections_[index]->tangent_angle(normalized_gamma) : std::nullopt;
}

/**
 * @brief Default method for computing the norm of the derivative 
 * @param gamma  The path parameter
 * @return std::optional<double> The norm of the derivative of the path position pd
 */
std::optional<double> Path::derivative_norm(double gamma) const {
    // Make the gamma vary between 0 and 1 for a given path section
    double normalized_gamma = gamma - std::floor(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    return (std::optional<unsigned int> index = get_section_index(gamma)) ? sections_[index]->derivative_norm(normalized_gamma) : std::nullopt;
}

/**
 * @brief Default method for getting the desired vehicle speed for a particular 
 * location in the path section (in m/s)
 * @param gamma The path parameter
 * @return std::optional<double> The desired vehicle speed (in m/s)
 */
std::optional<double> vehicle_speed(double gamma) const {
    // Make the gamma vary between 0 and 1 for a given path section
    double normalized_gamma = gamma - std::floor(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    return (std::optional<unsigned int> index = get_section_index(gamma)) ? sections_[index]->vehicle_speed(normalized_gamma) : std::nullopt;
}

/**
 * @brief Default method for getting the desired speed for the evolution
 * of the parametric value
 * @param The path parameter
 * @return std::optional<double> The desired speed progression of the parametric variable
 */
std::optional<double> Path::vd(double gamma) const {
    // Make the gamma vary between 0 and 1 for a given path section
    double normalized_gamma = gamma - std::floor(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    return (std::optional<unsigned int> index = get_section_index(gamma)) ? sections_[index]->vd(normalized_gamma) : std::nullopt;
}

/**
 * @brief Default method for getting the desired acceleration for the evolution
 * of the parametric value
 * @param gamma The path parameter
 * @return std::optional<double> The speed progression of the parametric variable
 */
std::optional<double> Path::d_vd(double gamma) const {
    // Make the gamma vary between 0 and 1 for a given path section
    double normalized_gamma = gamma - std::floor(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    return (std::optional<unsigned int> index = get_section_index(gamma)) ? sections_[index]->d_vd(normalized_gamma) : std::nullopt;
}

/**
 * @brief Method that outputs a bound parametric value between minimum and the maximum
 * parametric value that a path can achieve based on the maximum number of sections it contains
 * at any given time.
 * @param gamma The path parameter
 * @return double The bounded path parameter
 */
double Path::bound_gamma(double gamma) {
    // With substract a very small number because we can only access the vector
    // at the elements [0, number_sections[
    return std::min(std::max(0.0, gamma), (double) sections_.size() - 0.0000000001);
}

}