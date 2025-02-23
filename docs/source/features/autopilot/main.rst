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

.. toctree::
   :glob:
   :maxdepth: 4

   autopilot/autopilot
   controllers/controllers
   operation_modes/modes
   geofencing/geofencing
   trajectory_manager/trajectory_manager

