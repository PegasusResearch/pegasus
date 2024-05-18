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

.. code-block:: c++
   :linenos:
   :emphasize-lines: 1

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
