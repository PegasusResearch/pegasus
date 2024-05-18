Modes
======

In this section we provide an overview on the autopilot operating modes and how to add custom modes to the state machine. 
An operation mode is a state in which the autopilot can be in. The autopilot can be in only one mode at a time. 

This page is 
structure as follows:

* Section :ref:`0. Mode Interface` introduces the "Mode" class that defines the interface for the autopilot operating modes.
* Section :ref:`1. Provided State Machine Modes` provides an overview of the provided operating modes and where they are implemented.
* Section :ref:`2. Adding Custom Modes to the State Machine` provides a step-by-step guide on how to add custom operating modes to the state machine.

0. Mode Interface
-----------------

The interface for the autopilot operating modes is defined in The Autopilot is defined under the ``pegasus_autopilot/autopilot/include/autopilot/mode.hpp`` header file.
The ``Mode`` class is an abstract class that defines the interface for the autopilot operating modes. 

The autopilot operating modes must inherit from the ``Mode`` class and implement the following methods
highlighted in yellow in the code snippet below:

.. literalinclude:: ../../../pegasus_autopilot/autopilot/include/autopilot/mode.hpp
   :language: c++
   :emphasize-lines: 42-47
   :lines: 62-130
   :lineno-start: 1

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

2. Adding Custom Modes to the State Machine
-------------------------------------------

1. In this section we will implement a custom operating mode that drives the vehicle to a pre-defined waypoint.
Start by creating a new ROS 2 package inside your workspace, where you will implement the custom operating mode.

.. code:: bash

   # Go to the src directory of your workspace
   cd ~/pegasus/src

   # Create a C++ ROS 2 package where you are going to implement the custom modes
   ros2 pkg create --build-type ament_cmake custom_modes

   # Go inside the package directory
   cd custom_modes

   # Create a xml file where you are going to define your custom mode as a plugin
   touch custom_modes_plugins.xml

2. Inside the ``include/custom_modes`` directory, create a new header file named ``mode_fixed_waypoint.hpp``.

.. code:: bash

   # Create a new header file inside the include directory of the custom_modes package
   touch include/custom_modes/mode_fixed_waypoint.hpp

This file will contain the definition of the custom operating mode that drives the vehicle to a pre-defined waypoint. Your operation mode
must inherit from the ``autopilot::Mode`` class and implement the following methods: ``initialize``, ``enter``, ``exit`` and ``update``.

.. code-block:: c++
   :linenos:
   :emphasize-lines: 3, 7, 11-15

   #pragma once
   #include <Eigen/Core>
   #include <autopilot/mode.hpp>

   namespace autopilot {

   class FixedWaypointMode : public autopilot::Mode {

   public:

      ~FixedWaypointMode();
      void initialize() override;
      bool enter() override;
      bool exit() override;
      void update(double dt) override;

   protected:

      // The target position and attitude waypoint to be at
      Eigen::Vector3d target_pos{1.0, 1.0, -1.5};
      float target_yaw{0.0f};
   };
   }

3. Inside the ``src`` directory, create a new source file named ``mode_fixed_waypoint.cpp``.

.. code:: bash

   # Go to the src directory of the custom_modes package
   touch src/mode_fixed_waypoint.cpp


4. Modify the ``package.xml`` file to include the following dependencies ``autopilot`` and ``pluginlib``, according to the code snippet below.
This is necessary to let ROS 2 know that the package depends on the autopilot and pluginlib packages.

.. code-block:: xml
   :linenos:
   :emphasize-lines: 4-8, 12-13

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

      <test_depend>ament_lint_auto</test_depend>
      <test_depend>ament_lint_common</test_depend>

      <export>
         <build_type>ament_cmake</build_type>
      </export>
   </package>

5. Also modify the ``CMakeLists.txt`` file to include the following dependencies ``autopilot`` and ``pluginlib``, according to the code snippet below.

.. code-block:: cmake
   :linenos:
   :emphasize-lines: 9, 14-16, 19, 30-33, 37-38

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
   find_package(Eigen3 REQUIRED)

   add_library(${PROJECT_NAME} 
      src/mode_fixed_waypoint.cpp
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
   target_compile_definitions(${PROJECT_NAME} PRIVATE "AUTOPILOT_MODES_BUILDING_LIBRARY")

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
