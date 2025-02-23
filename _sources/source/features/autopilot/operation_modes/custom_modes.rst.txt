
Adding Custom Modes to the State Machine
========================================

In this section we will implement a custom waypoint operating mode. In this mode, we will subscribe to a ROS 2 topic that publishes the desired waypoint
and drive the vehicle to that waypoint. This is a simple example to show how to implement a custom operating mode. You can implement more complex modes
by following the same steps.

1. Start by creating a new ROS 2 package inside your workspace, where you will implement the custom operating mode.

.. code:: bash

   # Go to the src directory of your workspace
   cd ~/pegasus/src

   # Create a C++ ROS 2 package where you are going to implement the custom modes
   ros2 pkg create --build-type ament_cmake custom_modes

   # Go inside the package directory
   cd custom_modes

   # Create a xml file where you are going to define your custom mode as a plugin
   touch custom_modes_plugins.xml

2. Inside the ``include/custom_modes`` directory, create a new header file named ``mode_custom_waypoint.hpp``.

.. code:: bash

   # Create a new header file inside the include directory of the custom_modes package
   touch include/custom_modes/mode_custom_waypoint.hpp

This file will contain the definition of the custom operating mode that drives the vehicle to a pre-defined waypoint. Your operation mode
must inherit from the ``autopilot::Mode`` class and implement the following methods: ``initialize``, ``enter``, ``exit`` and ``update``.

.. code-block:: c++
   :linenos:
   :emphasize-lines: 3, 10, 14-18

   #pragma once
   #include <Eigen/Core>
   #include <autopilot/mode.hpp>
   #include "pegasus_msgs/msg/control_position.hpp"

   #include "rclcpp/rclcpp.hpp"

   namespace autopilot {

   class CustomWaypointMode : public autopilot::Mode {

   public:

      ~CustomWaypointMode();
      void initialize() override;
      bool enter() override;
      bool exit() override;
      void update(double dt) override;

   protected:

      // ROS 2 subscriber for the waypoint topic
      rclcpp::Subscription<pegasus_msgs::msg::ControlPosition>::SharedPtr waypoint_subscriber_;

      // Callback for the waypoint subscriber
      void waypoint_callback(const pegasus_msgs::msg::ControlPosition::ConstSharedPtr msg);

      // Stores the latest target position and attitude waypoint to be at
      Eigen::Vector3d target_pos{1.0, 1.0, -1.5};
      float target_yaw{0.0f};
      bool waypoint_set_{false};
   };
   }

* The ``initialize`` is called by the autopilot when it is loading all the operating modes into memory. This is where you should initialize the ROS 2 publishers, subscribers and services if needed.
* The ``enter`` is called by the autopilot when the mode is about to be entered. This is where you should check if the mode can be entered and set the necessary flags. If the mode cannot be entered, return false, and the autopilot will not enter this mode and will keep the current operation mode.
* The ``exit`` is called by the autopilot when the mode is about to be exited. This is where you should reset the flags and clean up any resources that were allocated.
* The ``update`` is called by the autopilot at every iteration of the control loop. This is where you should implement the logic of the mode.

3. Inside the ``src`` directory, create a new source file named ``mode_custom_waypoint.cpp``.

.. code:: bash

   # Go to the src directory of the custom_modes package
   touch src/mode_custom_waypoint.cpp


This file will contain the implementation of the custom operating mode that drives the vehicle to a given waypoint.

.. code-block:: c++
   :linenos:
   :emphasize-lines: 1, 10-11, 26-27, 42-43, 51-52, 61-62, 79-80

   #include "custom_modes/mode_custom_waypoint.hpp"

   namespace autopilot {

   CustomWaypointMode::~CustomWaypointMode() {
      // Terminate the waypoint subscriber
      this->waypoint_subscriber_.reset();
   }

   // This method is called when the autopilot loads all the modes into memory
   void CustomWaypointMode::initialize() {

      // Get the name of the topic where the waypoint is published from the ROS 2 parameter server
      node_->declare_parameter<std::string>("autopilot.CustomWaypointMode.waypoint_topic", "waypoint"); 

      // Create a subscriber to the waypoint topic (to get the desired waypoint to track)
      this->waypoint_subscriber_ = this->node_->create_subscription<pegasus_msgs::msg::ControlPosition>(
         node_->get_parameter("autopilot.CustomWaypointMode.waypoint_topic").as_string(),
         rclcpp::SystemDefaultsQoS(),
         std::bind(&CustomWaypointMode::waypoint_callback, this, std::placeholders::_1));
      
      // Log that the waypoint mode has been initialized correctly
      RCLCPP_INFO(this->node_->get_logger(), "CustomWaypointMode initialized");
   }

   // This method is called when the autopilot is about to enter the waypoint mode
   bool CustomWaypointMode::enter() {

      // Check if the waypoint was already set - if not, then do not enter the waypoint mode
      if (!this->waypoint_set_) {
         RCLCPP_ERROR(this->node_->get_logger(), "Waypoint not set - cannot enter waypoint mode.");
         return false;
      }

      // Reset the waypoint flag (to make sure we do not enter twice in this mode without setting a new waypoint)
      this->waypoint_set_ = false;

      // Return true to indicate that the mode has been entered successfully
      return true;
   }

   // This method is called when the autopilot is about to exit the waypoint mode and enter another mode
   bool CustomWaypointMode::exit() {

      this->waypoint_set_ = false;
      
      // Nothing to do here
      return true;   // Return true to indicate that the mode has been exited successfully
   }

   // This method is called at every iteration of the control loop by the autopilot
   void CustomWaypointMode::update(double dt) {

      // Set the controller to track the target position and attitude
      // In this case we do not which to implement a low-level controller, so we will use the set_position method from the controller interface
      // and use whatever controller is currently active in the autopilot.
      // Note: we could also have a custom implementation here and send lower level controls using the controller interface
      this->controller_->set_position(this->target_pos, this->target_yaw, dt);
   }

   // ROS 2 callback for the waypoint subscriber - this is called whenever a new waypoint is published
   void CustomWaypointMode::waypoint_callback(const pegasus_msgs::msg::ControlPosition::ConstSharedPtr msg) {
      
      // Set the waypoint
      this->target_pos[0] = msg->position[0];
      this->target_pos[1] = msg->position[1];
      this->target_pos[2] = msg->position[2];
      this->target_yaw = msg->yaw;

      // Set the waypoint flag
      this->waypoint_set_ = true;

      // Log that the waypoint has been set successfully
      RCLCPP_WARN(this->node_->get_logger(), "Waypoint set to (%f, %f, %f) with yaw %f", this->target_pos[0], this->target_pos[1], this->target_pos[2], this->target_yaw);
   }

   } // namespace autopilot

   #include <pluginlib/class_list_macros.hpp>
   PLUGINLIB_EXPORT_CLASS(autopilot::CustomWaypointMode, autopilot::Mode)

The last 2 lines of the code snippet above are necessary to export the custom mode as a plugin. The ``pluginlib`` package uses these lines to load the plugin at runtime.

4. Modify the ``package.xml`` file to include the following dependencies ``autopilot`` and ``pluginlib``, according to the code snippet below.
Additionally, we also need to include the ``pegasus_msgs`` package as a dependency, since we are subscribing to a ROS 2 topic that publishes messages from this package.

.. code-block:: xml
   :linenos:
   :emphasize-lines: 4-8, 12-14

   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
      <name>custom_modes</name>
      <version>1.0.0</version>
      <description></description>
      <author email="your.email@tecnico.ulisboa.pt">Author Name</author>
      <license>BSD-3</license>

      <buildtool_depend>ament_cmake_ros</buildtool_depend>

      <depend>autopilot</depend>
      <depend>pluginlib</depend>
      <depend>pegasus_msgs</depend>

      <test_depend>ament_lint_auto</test_depend>
      <test_depend>ament_lint_common</test_depend>

      <export>
         <build_type>ament_cmake</build_type>
      </export>
   </package>

5. Also modify the ``CMakeLists.txt`` file to include the following dependencies ``autopilot``, ``pegasus_msgs`` and ``pluginlib``, according to the code snippet below. Additionally, we need to include the ``Eigen3`` package as a dependency, since we are using Eigen for linear algebra operations.

.. code-block:: cmake
   :linenos:
   :emphasize-lines: 9, 13-17, 19-21, 31-35, 39-40, 50-52

   cmake_minimum_required(VERSION 3.8)
   project(custom_modes)

   # Default to C++20 and compiler flags to give all warnings
   set(CMAKE_CXX_STANDARD 20)
   set(CMAKE_CXX_STANDARD_REQUIRED ON)

   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
   add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-sign-compare -O3)
   endif()

   # find dependencies
   find_package(ament_cmake REQUIRED)
   find_package(autopilot REQUIRED)
   find_package(pluginlib REQUIRED)
   find_package(pegasus_msgs REQUIRED)
   find_package(Eigen3 REQUIRED)

   add_library(${PROJECT_NAME} 
      src/mode_custom_waypoint.cpp
   )

   target_include_directories(${PROJECT_NAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${EIGEN3_INCLUDE_DIRS}
   )

   add_definitions(${EIGEN3_DEFINITIONS})

   set(dependencies 
      autopilot
      pluginlib
      pegasus_msgs
   )

   ament_target_dependencies(${PROJECT_NAME} ${dependencies})

   # Export the pluginlib description (package containing the base class and the derived classes information in XML format)
   pluginlib_export_plugin_description_file(autopilot custom_modes_plugins.xml)

   install(
      TARGETS ${PROJECT_NAME}
      EXPORT export_${PROJECT_NAME}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
   )

   # Causes the visibility macros to use dllexport rather than dllimport,
   # which is appropriate when building the dll but not consuming it.
   target_compile_definitions(${PROJECT_NAME} PRIVATE "CUSTOM_MODES_BUILDING_LIBRARY")

   install(
      DIRECTORY include/
      DESTINATION include
   )

   if(BUILD_TESTING)
   find_package(ament_lint_auto REQUIRED)
   # the following line skips the linter which checks for copyrights
   # comment the line when a copyright and license is added to all source files
   set(ament_cmake_copyright_FOUND TRUE)
   # the following line skips cpplint (only works in a git repo)
   # comment the line when this package is in a git repo and when
   # a copyright and license is added to all source files
   set(ament_cmake_cpplint_FOUND TRUE)
   ament_lint_auto_find_test_dependencies()
   endif()

   ament_export_include_directories(include)
   ament_export_libraries(${PROJECT_NAME})
   ament_export_dependencies(${dependencies})
   ament_export_targets(export_${PROJECT_NAME})
   ament_package()

6. Create a xml file named ``custom_modes_plugins.xml`` inside the package directory. This file will contain the definition of the custom mode as a plugin.

.. code:: bash

   # Create a xml file inside the package directory
   touch custom_modes_plugins.xml

The content of the ``custom_modes_plugins.xml`` file should be as follows:

.. code-block:: xml
   :linenos:

   <library path="custom_modes">
      <class type="autopilot::CustomWaypointMode" base_class_type="autopilot::Mode">
         <description>A custom waypoint mode that drives the vehicle to a given waypoint.</description>
      </class>
   </library>

7. Create a configuration file named ``custom_modes.yaml`` inside the package directory. This file will contain the configuration parameters for launching the autopilot
with the custom mode.

.. code:: bash

   # Create a yaml file inside the package directory
   touch config/custom_modes.yaml

The content of the ``custom_modes.yaml`` file should be as follows. In line 8, we define the modes that will be loaded by the autopilot. In this case, we are adding the custom waypoint mode to the list of modes.

Lines 23-26 define the configuration parameters for each mode, including the newly created custom waypoint mode. In this case, we are defining the topic where the waypoint is published.

We also define the valid_transitions, which define which modes are allowed to transition to from the custom waypoint mode. In this case, the custom waypoint mode can transition to the ``Hold`` and ``Land`` modes.
Moreover we define the fallback mode, which is the mode that the autopilot will transition to if the custom waypoint mode is not able to execute its logic.

.. code-block:: yaml
   :linenos:
   :emphasize-lines: 8, 23-26

   /**:
   ros__parameters:
      autopilot:
         # Update rate
         rate: 50.0 # Hz
         default_mode: "DisarmMode"
         # Define all the existing operation modes
         modes: ["DisarmMode", "ArmMode", "LandMode", "HoldMode", "CustomWaypointMode"]
         # Configurations of each operating mode:
         DisarmMode: 
            valid_transitions: ["ArmMode"]
            fallback: "DisarmMode"
            disarm_service: "fmu/kill_switch"
         ArmMode: 
            valid_transitions: ["DisarmMode", "CustomWaypointMode"]
            fallback: "DisarmMode"
            geofencing_violation_fallback: "DisarmMode"
            arm_service: "fmu/arm"
            offboard_service: "fmu/offboard"
         HoldMode: 
            valid_transitions: ["LandMode", "CustomWaypointMode"]
            fallback: "LandMode"
         LandMode: 
            valid_transitions: ["DisarmMode", "ArmMode", "TakeoffMode", "HoldMode", "WaypointMode", "FollowTrajectoryMode", "PassThroughMode"]
            fallback: "HoldMode"
            on_finish: "DisarmMode"
            land_speed: 0.2 # m/s
            land_detected_treshold: 0.1 # m/s
            countdown_to_disarm: 3.0 # s
         CustomWaypointMode:
            valid_transitions: ["HoldMode", "LandMode"]
            fallback: "HoldMode"
            waypoint_topic: "waypoint"
         # Topics configurations for the autopilot to communicate with the rest of the system
         publishers:
            control_position: "fmu/in/position"
            control_attitude: "fmu/in/force/attitude"
            control_attitude_rate: "fmu/in/force/attitude_rate"
            status: "autopilot/status"
         subscribers:
            state: "fmu/filter/state"
            status: "fmu/status"
            constants: "fmu/constants"
         services:
            set_mode: "autopilot/change_mode"
         # ----------------------------------------------------------------------------------------------------------
         # Definition of the controller that will perform the tracking of references of the different operation modes
         # ----------------------------------------------------------------------------------------------------------
         controller: "OnboardController"
         OnboardController:
            publishers:
               control_position: "fmu/in/position"
               control_attitude: "fmu/in/force/attitude"
               control_attitude_rate: "fmu/in/force/attitude_rate"
         # ----------------------------------------------------------------------------------------------------------
         # Definition of the geofencing mechanism that will keep the vehicle in safe places
         # ----------------------------------------------------------------------------------------------------------
         geofencing: "BoxGeofencing"
         BoxGeofencing:
            limits_x: [-10.0, 10.0]
            limits_y: [-10.0, 10.0]
            limits_z: [-10.0,  1.0] # NED Coordinades (z-negative is up)
         # ----------------------------------------------------------------------------------------------------------
         # Definition of the trajectory manager that generates parameterized trajectories to be followed
         # ----------------------------------------------------------------------------------------------------------
         trajectory_manager: "StaticTrajectoryManager"
         StaticTrajectoryManager:
            services:
               reset_trajectory: "autopilot/trajectory/reset"

8. Create a launch file named ``custom_modes.launch.py`` inside the package directory. This file will launch the autopilot with the configurations provided in the ``custom_modes.yaml`` file.

.. code:: bash

   # Create a launch file inside the package directory
   touch launch/custom_modes.launch.py

The content of the ``custom_modes.launch.py`` file should be as follows:

.. code-block:: python
   :linenos:

   from launch import LaunchDescription
   from launch_ros.actions import Node
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration

   def generate_launch_description():
      return LaunchDescription([
         DeclareLaunchArgument(
            'config_file',
            default_value='custom_modes.yaml',
            description='Path to the configuration file for the autopilot'
         ),
         Node(
            package='autopilot',
            executable='autopilot_node',
            name='autopilot',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
         )
      ])