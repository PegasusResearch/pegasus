Autopilot
=========

At the center of the Guidance and Control system is the Autopilot, implemented in C++. This is located inside the ``pegasus_autopilot/autopilot`` package. 
The Autopilot is responsible for keeping track of the current operating ``mode`` of the vehicle. It exposes to each operation ``mode`` the following APIs:

* ``Control`` - Provides the control commands to the vehicle to track a given position, attitude, velocity, etc.
* ``Geofencing`` - Checks if the vehicle is within the geofence and if not, provides the control commands to bring the vehicle back inside the geofence.
* ``Trajectory Manager`` - Manages desired trajectories for the vehicle to follow.

It also provides each mode with the current ``state`` of the vehicle, which includes the current position, velocity, attitude, etc.

The autopilot operation ``modes``, ``controllers``, ``geofencing`` and ``trajectory manager`` are implemented as ROS 2 plugins. This allows for easy extensibility and customization of the Autopilot, without having
to modify the base autopilot packages. The Autopilot is responsible for loading the plugins at runtime and managing the communication between them.

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


1. Autopilot Code
-----------------

.. literalinclude:: ../../../pegasus_autopilot/autopilot/include/autopilot/autopilot.hpp
   :language: c++
   :emphasize-lines: 87-88, 90-91, 93-94, 102-103, 127-128, 130-131, 133-136, 182-186 
   :linenos:

2. Explanation
--------------

In order to 

.. literalinclude:: ../../../pegasus_autopilot/autopilot/include/autopilot/autopilot.hpp
   :language: c++
   :lines: 87-88
   :lineno-start: 87





