#include "pegasus_utils/rotations.hpp"
#include "autopilot_modes/mode_follow_trajectory.hpp"
#include "thrust_curves/acceleration_to_attitude.hpp"

namespace autopilot {

FollowTrajectoryMode::~FollowTrajectoryMode() {}

void FollowTrajectoryMode::initialize() {
    
    // Load the controller gains from the parameter server
    node_->declare_parameter<std::vector<double>>("autopilot.FollowTrajectoryMode.gains.kp", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.FollowTrajectoryMode.gains.kd", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.FollowTrajectoryMode.gains.ki", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.FollowTrajectoryMode.gains.kff", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.FollowTrajectoryMode.gains.min_output", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.FollowTrajectoryMode.gains.max_output", std::vector<double>());

    auto kp = node_->get_parameter("autopilot.FollowTrajectoryMode.gains.kp").as_double_array();
    auto kd = node_->get_parameter("autopilot.FollowTrajectoryMode.gains.kd").as_double_array();
    auto ki = node_->get_parameter("autopilot.FollowTrajectoryMode.gains.ki").as_double_array();
    auto kff = node_->get_parameter("autopilot.FollowTrajectoryMode.gains.kff").as_double_array();
    auto min_output = node_->get_parameter("autopilot.FollowTrajectoryMode.gains.min_output").as_double_array();
    auto max_output = node_->get_parameter("autopilot.FollowTrajectoryMode.gains.max_output").as_double_array();

    // Safety check on the gains (make sure they are there)
    if(kp.size() != 3 || kd.size() != 3 || ki.size() != 3 || kff.size() != 3 || min_output.size() != 3 || max_output.size() != 3) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not read PID position controller gains correctly.");
        throw std::runtime_error("Gains vector was empty");
    }

    // Create the 3 PID controllers for x, y and z axis
    for(unsigned int i=0; i < 3; i++) controllers_[i] = std::make_unique<Pegasus::Pid>(kp[i], kd[i], ki[i], kff[i], min_output[i], max_output[i]);

    // Get the mass of the vehicle (used to get the thrust from the acceleration)
    VehicleConstants vehicle_constansts = get_vehicle_constants();
    mass_ = vehicle_constansts.mass;

    // ----------------------- TRAJECTORY SETUP -------------------------

    // Load the topics for the trajectories
    node_->declare_parameter<std::string>("autopilot.FollowTrajectoryMode.reset_path_topic", "path/reset");
    node_->declare_parameter<std::string>("autopilot.FollowTrajectoryMode.add_arc_topic", "path/add_arc");
    node_->declare_parameter<std::string>("autopilot.FollowTrajectoryMode.add_line_topic", "path/add_line");
    node_->declare_parameter<std::string>("autopilot.FollowTrajectoryMode.add_circle_topic", "path/add_circle");
    node_->declare_parameter<std::string>("autopilot.FollowTrajectoryMode.add_lemniscate_topic", "path/add_lemniscate");
    node_->declare_parameter<std::string>("autopilot.FollowTrajectoryMode.sampled_path_topic", "path/points");
    node_->declare_parameter<std::string>("autopilot.FollowTrajectoryMode.pid_debug_topic", "statistics/pid");

    reset_service_ = node_->create_service<pegasus_msgs::srv::ResetPath>(node_->get_parameter("autopilot.FollowTrajectoryMode.reset_path_topic").as_string(), std::bind(&FollowTrajectoryMode::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    add_arc_service_ = node_->create_service<pegasus_msgs::srv::AddArc>(node_->get_parameter("autopilot.FollowTrajectoryMode.add_arc_topic").as_string(), std::bind(&FollowTrajectoryMode::add_arc_callback, this, std::placeholders::_1, std::placeholders::_2));
    add_line_service_ = node_->create_service<pegasus_msgs::srv::AddLine>(node_->get_parameter("autopilot.FollowTrajectoryMode.add_line_topic").as_string(), std::bind(&FollowTrajectoryMode::add_line_callback, this, std::placeholders::_1, std::placeholders::_2));
    add_circle_service_ = node_->create_service<pegasus_msgs::srv::AddCircle>(node_->get_parameter("autopilot.FollowTrajectoryMode.add_circle_topic").as_string(), std::bind(&FollowTrajectoryMode::add_circle_callback, this, std::placeholders::_1, std::placeholders::_2));
    add_lemniscate_service_ = node_->create_service<pegasus_msgs::srv::AddLemniscate>(node_->get_parameter("autopilot.FollowTrajectoryMode.add_lemniscate_topic").as_string(), std::bind(&FollowTrajectoryMode::add_lemniscate_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    // Initialize the publishers for the statistics of the PID controller and the path to follow
    points_pub_ = node_->create_publisher<nav_msgs::msg::Path>(node_->get_parameter("autopilot.FollowTrajectoryMode.sampled_path_topic").as_string(), 1);
    statistics_pub_ = node_->create_publisher<pegasus_msgs::msg::PidStatistics>(node_->get_parameter("autopilot.FollowTrajectoryMode.pid_debug_topic").as_string(), 1);

    RCLCPP_INFO(this->node_->get_logger(), "FollowTrajectoryMode initialized");
}

bool FollowTrajectoryMode::enter() {

    // Check if the path is empty. If it is, return with error
    if(path_.empty()) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Path is empty. Cannot follow an empty path.");
        return false;
    }

    // Otherwise, enter the trajectory following mode
    return true;
}

bool FollowTrajectoryMode::exit() {
    
    // Reset the parametric values
    gamma_ = 0.0;
    d_gamma_ = 0.0;
    dd_gamma_ = 0.0;

    // Reset the desired targets
    desired_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_acceleration_ = Eigen::Vector3d(0.0, 0.0, 0.0);

    // Reset the controllers
    for(unsigned int i=0; i < 3; i++) controllers_[i]->reset_controller();

    // Return with success
    return true;
}

void FollowTrajectoryMode::update_reference(double dt) {

    // Get the desired speed progression of the path at the current location
    d_gamma_ = path_.vd(gamma_).value();
    dd_gamma_ = path_.d_vd(gamma_).value();

    // Update the desired position, velocity and acceleration from the path
    desired_position_ = path_.pd(gamma_).value();
    desired_velocity_ = path_.d_pd(gamma_).value() * d_gamma_;
    desired_acceleration_ = (path_.dd_pd(gamma_).value() * std::pow(d_gamma_, 2)) + (path_.d_pd(gamma_).value() * std::pow(dd_gamma_, 2));

    // Update the desired yaw from the tangent to the path
    // TODO: make the this more general later on
    desired_yaw_ = 0.0; //path_.tangent_angle(gamma_).value();

    // Integrate the parametric value (virtual target) over time
    gamma_ += d_gamma_ * dt;

    // Saturate the parametric value
    gamma_ = std::min(std::max(path_.get_min_gamma(), gamma_), path_.get_max_gamma());
}

bool FollowTrajectoryMode::check_finished() {

    // Get the stats from the PID controllers
    // Pegasus::Pid::Statistics stats_x = controllers_[0]->get_statistics();
    // Pegasus::Pid::Statistics stats_y = controllers_[1]->get_statistics();
    // Pegasus::Pid::Statistics stats_z = controllers_[2]->get_statistics();

    // Check if the path is finished
    // if(gamma_ >= path_.get_max_gamma() &&  stats_x.error_p < 0.1 && stats_y.error_p < 0.1 && stats_z.error_p < 0.1) {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "Trajectory Tracking mission finished.");
    //     signal_mode_finished();
    //     return true;
    // }

    return false;
}

void FollowTrajectoryMode::update_statistics() {
    
    // For each PID control [x, y, z]
    for(unsigned int i = 0; i < 3; i++) {

        // Get the statistics from the controller object
        Pegasus::Pid::Statistics stats = controllers_[i]->get_statistics();

        pid_statistics_msg_.statistics[i].dt = stats.dt;
        pid_statistics_msg_.statistics[i].reference = desired_position_[i];
        // Fill the feedback errors
        pid_statistics_msg_.statistics[i].error_p = stats.error_p;
        pid_statistics_msg_.statistics[i].error_d = stats.error_d;
        pid_statistics_msg_.statistics[i].integral = stats.integral;
        pid_statistics_msg_.statistics[i].ff_ref = stats.ff_ref;

        // Fill the errors scaled by the gains
        pid_statistics_msg_.statistics[i].p_term = stats.p_term;
        pid_statistics_msg_.statistics[i].d_term = stats.d_term;
        pid_statistics_msg_.statistics[i].i_term = stats.i_term;
        pid_statistics_msg_.statistics[i].ff_term = stats.ff_term;

        // Fill the outputs of the controller
        pid_statistics_msg_.statistics[i].anti_windup_discharge = stats.anti_windup_discharge;
        pid_statistics_msg_.statistics[i].output_pre_sat = stats.output_pre_sat;
        pid_statistics_msg_.statistics[i].output = stats.output;
    }
}

void FollowTrajectoryMode::update(double dt) {

    // Update the current reference on the path to follow
    update_reference(dt);

    // Get the current state of the vehicle
    State state = get_vehicle_state();
    
    // Compute the position error and velocity error using the path desired position and velocity
    Eigen::Vector3d pos_error = desired_position_ - state.position;
    Eigen::Vector3d vel_error = desired_velocity_ - state.velocity;
    Eigen::Vector3d accel = desired_acceleration_;

    // Compute the desired control output acceleration for each controller
    Eigen::Vector3d u;
    const Eigen::Vector3d g(0.0, 0.0, 9.81);
    for(unsigned int i=0; i < 3; i++) u[i] = controllers_[i]->compute_output(pos_error[i], vel_error[i], (accel[i] * mass_) - g[i], dt);
    
    // Convert the acceleration to attitude and thrust
    Eigen::Vector4d attitude_thrust = get_attitude_thrust_from_acceleration(u, mass_, desired_yaw_);

    // Set the control output
    Eigen::Vector3d attitude_target = Eigen::Vector3d(
        Pegasus::Rotations::rad_to_deg(attitude_thrust[0]), 
        Pegasus::Rotations::rad_to_deg(attitude_thrust[1]), 
        Pegasus::Rotations::rad_to_deg(attitude_thrust[2]));
    set_attitude(attitude_target, attitude_thrust[3]);

    // Update and publish the PID statistics
    update_statistics();
    statistics_pub_->publish(pid_statistics_msg_);

    // Check if we have reached the end of the path
    check_finished();
}

// Services callbacks to set the path to follow
void FollowTrajectoryMode::reset_callback(const pegasus_msgs::srv::ResetPath::Request::SharedPtr, const pegasus_msgs::srv::ResetPath::Response::SharedPtr response) {

    RCLCPP_INFO_STREAM(node_->get_logger(), "Resetting path.");

    // Clear the path
    path_.clear();

    // Call the controller reset function if one is running to signal that the path was cleaned
    for(unsigned int i=0; i < 3; i++) controllers_[i]->reset_controller();

    // Reset the path points message
    path_points_msg_.header.frame_id = "world_ned";
    path_points_msg_.header.stamp = node_->get_clock()->now();

    // Clear the points message vector (such that RVIZ shows a clear path)
    path_points_msg_.poses.clear();
    points_pub_->publish(path_points_msg_);

    // Make the response of this service to true
    response->success = true;
    RCLCPP_INFO_STREAM(node_->get_logger(), "Path reset");
}

void FollowTrajectoryMode::add_section_to_path(const Pegasus::Paths::Section::SharedPtr section) {
    
    // Add the new section to the path
    path_.push_back(section);

    // Update the samples of the path
    auto path_samples = path_.get_samples(sample_step_);

    // If the path samples optional is not null, update the message that describes the path
    if(path_samples.has_value()) {

        // Set the header of the message
        path_points_msg_.header.frame_id = "world_ned";
        path_points_msg_.header.stamp = node_->get_clock()->now();

        // Clear the points message vector
        path_points_msg_.poses.clear();

        // Create a new vector with new samples
        for (auto & sample: path_samples.value()) {
            
            // Create a pose message object
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = sample[0];
            pose.pose.position.y = sample[1];
            pose.pose.position.z = sample[2];

            // Add it to the list of points message
            path_points_msg_.poses.push_back(pose);
        }   

        // Publish the points message
        points_pub_->publish(path_points_msg_);
    }
}

void FollowTrajectoryMode::add_arc_callback(const pegasus_msgs::srv::AddArc::Request::SharedPtr request, const pegasus_msgs::srv::AddArc::Response::SharedPtr response) {

    // Log the parameters of the path section to be added
    RCLCPP_INFO_STREAM(node_->get_logger(), "Adding arc to path. Speed: " << request->speed.parameters[0] << ", start: [" << request->start[0] << "," << request->start[1] << "], center: [" << request->center[0] << "," << request->center[1] << "," << request->center[2] << "], normal: [" << request->normal[0] << "," << request->normal[1] << "," << request->normal[2] << "], clockwise: " << request->clockwise_direction << ".");

    // Create a new Speed object (by default just get the value for now - TO BE IMPROVED)
    auto speed = std::make_shared<Pegasus::Paths::ConstSpeed>(request->speed.parameters[0]);

    // Create a new Arc object
    auto arc = std::make_shared<Pegasus::Paths::Arc>(speed, Eigen::Vector2d(request->start.data()), Eigen::Vector3d(request->center.data()), Eigen::Vector3d(request->normal.data()), request->clockwise_direction);

    // Add the new arc to the path
    add_section_to_path(arc);

    // Update the response
    response->success = true;
}

void FollowTrajectoryMode::add_line_callback(const pegasus_msgs::srv::AddLine::Request::SharedPtr request, const pegasus_msgs::srv::AddLine::Response::SharedPtr response) {

    // Log the parameters of the path section to be added
    RCLCPP_INFO_STREAM(node_->get_logger(), "Adding line to path. Speed: " << request->speed.parameters[0] << ", start: [" << request->start[0] << "," << request->start[1] << "," << request->start[2] << "], end: [" << request->end[0] << "," << request->end[1] << "," << request->end[2] << "].");

    // Create a new Speed object (by default just get the value for now - TO BE IMPROVED)
    auto speed = std::make_shared<Pegasus::Paths::ConstSpeed>(request->speed.parameters[0]);

    // Create a new Line object
    auto line = std::make_shared<Pegasus::Paths::Line>(speed, Eigen::Vector3d(request->start.data()), Eigen::Vector3d(request->end.data()));

    // Add the new line to the path
    add_section_to_path(line);

    // Update the response
    response->success = true;
}

void FollowTrajectoryMode::add_circle_callback(const pegasus_msgs::srv::AddCircle::Request::SharedPtr request, const pegasus_msgs::srv::AddCircle::Response::SharedPtr response) {

    // Log the parameters of the path section to be added
    RCLCPP_INFO_STREAM(node_->get_logger(), "Adding circle to path. Speed: " << request->speed.parameters[0] << ", center: [" << request->center[0] << "," << request->center[1] << "," << request->center[2] << "], normal: [" << request->normal[0] << "," << request->normal[1] << "," << request->normal[2] << "], radius: " << request->radius << ".");

    // Create a new Speed object (by default just get the value for now - TO BE IMPROVED)
    auto speed = std::make_shared<Pegasus::Paths::ConstSpeed>(request->speed.parameters[0]);

    // Create a new Circle object
    auto circle = std::make_shared<Pegasus::Paths::Circle>(speed, Eigen::Vector3d(request->center.data()), Eigen::Vector3d(request->normal.data()), request->radius);

    // Add the new circle to the path
    add_section_to_path(circle);

    // Update the response
    response->success = true;
}

void FollowTrajectoryMode::add_lemniscate_callback(const pegasus_msgs::srv::AddLemniscate::Request::SharedPtr request, const pegasus_msgs::srv::AddLemniscate::Response::SharedPtr response) {

    // Log the parameters of the path section to be added
    RCLCPP_INFO_STREAM(node_->get_logger(), "Adding lemniscate to path. Speed: " << request->speed.parameters[0] << ", center: [" << request->center[0] << "," << request->center[1] << "," << request->center[2] << "], normal: [" << request->normal[0] << "," << request->normal[1] << "," << request->normal[2] << "], radius: " << request->radius << ".");
    
    // Create a new Speed object (by default just get the value for now - TO BE IMPROVED)
    auto speed = std::make_shared<Pegasus::Paths::ConstSpeed>(request->speed.parameters[0]);

    // Create a new Lemniscate object
    auto lemniscate = std::make_shared<Pegasus::Paths::Lemniscate>(speed, Eigen::Vector3d(request->center.data()), Eigen::Vector3d(request->normal.data()), request->radius);

    // Add the new circle to the path
    add_section_to_path(lemniscate);

    // Update the response
    response->success = true;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::FollowTrajectoryMode, autopilot::Mode)