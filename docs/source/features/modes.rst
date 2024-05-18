Modes
======

An operation ``mode`` is a state in which the autopilot can be in. The autopilot can be in only one mode at a time.
In this section we provide an overview on the autopilot operating modes and how to add custom modes to the state machine.

Depending on the target application, an autopilot ``mode`` can be used to define a new behaviour for the vehicle to execute,
or be used to prototype a controller as well. For instance, if you want to prototype a new controller to be used only for following
a specific trajectory, then it might make sense to implement it as a ``mode`` rather than a ``controller`` that is used by all the other available modes.

This page is structured as follows:

* Section :ref:`0. Mode Interface` introduces the "Mode" class that defines the interface for the autopilot operating modes.
* Section :ref:`1. Provided State Machine Modes` provides an overview of the provided operating modes and where they are implemented.
* Section :ref:`2. Adding Custom Modes to the State Machine` provides a step-by-step guide on how to add custom operating modes to the state machine.

0. Mode Interface
-----------------

The interface for the autopilot operating modes is defined in the autopilot package under the ``pegasus_autopilot/autopilot/include/autopilot/mode.hpp`` header file.
The ``Mode`` class is an abstract class that defines the interface for the autopilot operating modes. 

The autopilot operating modes must inherit from the ``Mode`` class and implement the methods
highlighted in yellow (lines 42-47) in the code snippet below:

.. literalinclude:: ../../../pegasus_autopilot/autopilot/include/autopilot/mode.hpp
   :language: c++
   :emphasize-lines: 42-47
   :lines: 62-130
   :lineno-start: 1

Additionally, the ``Mode`` class provides methods to get:

1. The current ``State`` of the vehicle can be accessed by calling ``get_vehicle_state()`` . It returns the following struct:

.. literalinclude:: ../../../pegasus_autopilot/autopilot/include/autopilot/state.hpp
   :language: c++
   :lines: 52,54-60,77
   :lineno-start: 1

2. The ``Status`` of the vehicle can be accessed by calling ``get_vehicle_status()`` . It returns the following struct:  

.. literalinclude:: ../../../pegasus_autopilot/autopilot/include/autopilot/state.hpp
   :language: c++
   :lines: 52,62-67,77
   :lineno-start: 1

3. The ``Constants`` such as mass or thrust curve can be accessed by calling ``get_vehicle_constants`` . It returns the following struct:

.. literalinclude:: ../../../pegasus_autopilot/autopilot/include/autopilot/state.hpp
   :language: c++
   :lines: 52,69-75,77
   :lineno-start: 1

This means that to access these values, you do not need to manually subscribe to the ROS 2 topics. The autopilot
takes care of that for you. However, if you are required to access other values that are not provided by the ``Mode`` class, or if you need to publish
values to the ROS 2 topics, you can do so by using the ``node_`` shared pointer that is a member of the ``Mode`` class.

4. The ``Mode`` class also provides a shared pointer to a ``controller_`` object that can be used to access the controller interface. To check
the available methods in the controller interface, please refer to the :ref:`Controllers` page. This is useful if you want to send control commands to the vehicle
without having to publish to the ROS 2 topics directly. The autopilot takes care of that for you.

5. The ``Mode`` class also provides a shared pointer to a ``trajectory_manager_`` object that can be used to access the trajectory manager interface. To check
the available methods in the trajectory manager interface, please refer to the :ref:`Trajectory Manager` page. This is useful if your mode needs to follow a trajectory
and you don't want to manually subscribe to the ROS 2 topics or hard-code the trajectory in the mode.

1. Provided State Machine Modes
-------------------------------
The autopilot provides the following operating modes:

* ``Arm`` - The vehicle is armed and ready to take off.
* ``Disarm`` - The vehicle is on the ground and disarmed.
* ``Takeoff`` - The vehicle takes of to a predefined altitude above the current position.
* ``Land`` - The vehicle lands at the current position.
* ``Hold`` - The vehicle holds its current position.
* ``Follow Trajectory`` - The vehicle follows the trajectory loaded in the trajectory manager.
* ``Waypoint`` - The vehicle goes to a waypoint.

Since the autopilot modes are implemented as plugins, they are loaded at runtime by the autopilot. The autopilot uses the ``pluginlib`` package to load the plugins.
This also allows for the modes to be implemented in different packages, which is useful if you want to keep the autopilot package clean and modular.

For instance, if you want to check the implementation of the provided base modes, you can find them in the ``pegasus_autopilot/autopilot_modes`` package.

2. Adding Custom Modes to the State Machine
-------------------------------------------

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

The content of the ``custom_modes.yaml`` file should be as follows:

.. code-block:: yaml
   :linenos:

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
