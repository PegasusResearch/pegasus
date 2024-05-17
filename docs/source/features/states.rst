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
TODO