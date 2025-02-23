Provided State Machine Modes
============================

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