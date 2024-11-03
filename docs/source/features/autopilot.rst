Autopilot
=========

At the center of the Guidance and Control system is the Autopilot, implemented in C++. This is located inside the ``pegasus_autopilot/autopilot`` package. 
The Autopilot is responsible for keeping track of the current operating ``mode`` of the vehicle. It exposes to each operation ``mode`` the following APIs:

* ``Control`` - Provides the control commands to the vehicle to track a given position, attitude, velocity, etc.
* ``Geofencing`` - Checks if the vehicle is within the geofence and if not, provides the control commands to bring the vehicle back inside the geofence.
* ``Trajectory Manager`` - Manages desired trajectories for the vehicle to follow.

It also provides each mode with the current ``state`` of the vehicle, which includes the current position, velocity, attitude, etc.

.. image:: /_static/features/autopilot/autopilot_architecture.png
            :width: 400px
            :align: center
            :alt: Autopilot Architecture

The autopilot operation ``modes``, ``controllers``, ``geofencing`` and ``trajectory manager`` are implemented as ROS 2 plugins. This allows for easy extensibility and customization of the Autopilot, without having
to modify the base autopilot packages. The Autopilot is responsible for loading the plugins at runtime and managing the communication between them.


.. mermaid::
   
   classDiagram
      pegasus <|-- pegasus_interfaces
      pegasus <|-- pegasus_console
      pegasus <|-- pegasus_autopilot
      pegasus <|-- pegasus_api
      pegasus_autopilot <|-- autopilot_modes
      pegasus_autopilot <|-- autopilot_controllers
      pegasus_autopilot <|-- geofencing
      pegasus_autopilot <|-- trajectory_manager
      trajectory_manager <|-- static_trajectory_manager
      static_trajectory_manager <|-- static_trajectories
      geofencing <|-- box_geofencing
      pegasus_interfaces <|-- mocap_interface
      pegasus_interfaces <|-- mavlink_interface
      class pegasus{
         <<Meta Package>>
         VehicleLaunchFiles
         VehicleConfigurationFiles
      }
      class mocap_interface{
         <<C++ Package>>
      }
      class pegasus_interfaces{
         <<Meta Package>>
      }
      class mavlink_interface{
         <<C++ Package>>
      }
      class pegasus_console{
         <<C++ Package>>  
      }
      class pegasus_autopilot{
         <<C++ Package>>
      }
      class autopilot_controllers{
         <<C++ Package>>
      }
      class autopilot_modes{
         <<C++ Package>>
      }
      class pegasus_api{
         <<Python Package>>
      }
      class trajectory_manager{
         <<C++ Package>>
      }

0. Operating Modes
------------------

The default operating modes provided by the Autopilot are:

* ``Arm`` - The vehicle is armed and ready to take off.
* ``Disarm`` - The vehicle is on the ground and disarmed.
* ``Takeoff`` - The vehicle takes of to a predefined altitude above the current position.
* ``Land`` - The vehicle lands at the current position.
* ``Hold`` - The vehicle holds its current position.
* ``Follow Trajectory`` - The vehicle follows the trajectory loaded in the trajectory manager.
* ``Waypoint`` - The vehicle goes to a waypoint.


1. Autopilot Interface
----------------------

The Autopilot is defined in the under the ``pegasus_autopilot/autopilot/include/autopilot/autopilot.hpp`` header file.

.. literalinclude:: ../../../pegasus_autopilot/autopilot/include/autopilot/autopilot.hpp
   :language: c++
   :emphasize-lines: 87-88
   :lines: 78-189
   :lineno-start: 1

2. Explanation
--------------

In order to 

.. literalinclude:: ../../../pegasus_autopilot/autopilot/include/autopilot/autopilot.hpp
   :language: c++
   :lines: 87-88
   :lineno-start: 87

3. Autopilot Configuration File
-------------------------------

In order to 

.. literalinclude:: ../../../pegasus_autopilot/autopilot/config/autopilot.yaml
   :language: yaml
   :lines: 1-40
   :lineno-start: 1

.. literalinclude:: ../../../pegasus_autopilot/autopilot/config/autopilot.yaml
   :language: yaml
   :lines: 41-48
   :lineno-start: 41

.. literalinclude:: ../../../pegasus_autopilot/autopilot/config/autopilot.yaml
   :language: yaml
   :lines: 49-67
   :lineno-start: 49

.. literalinclude:: ../../../pegasus_autopilot/autopilot/config/autopilot.yaml
   :language: yaml
   :lines: 68-100
   :lineno-start: 68