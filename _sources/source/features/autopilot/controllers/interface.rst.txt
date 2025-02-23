Controller Interface
====================

The interface for the autopilot controller is defined in the autopilot package under the ``pegasus_autopilot/autopilot/include/autopilot/controller.hpp`` header file. 
The ``Controller`` class is an abstract class that defines the interface for the autopilot controller.

Each autopilot controller **must** inherit from the ``Controller`` class and **implement the following one or multiple methods highlighted in yellow** in the code snippet below:

.. literalinclude:: ../../../../../pegasus_autopilot/autopilot/include/autopilot/controller.hpp
   :language: c++
   :emphasize-lines: 31-34, 36-40, 42-49, 51-53, 55-57, 59-61, 63-65, 67-69, 71-78, 80-87, 89-96, 98-105, 107-112
   :lines: 62-186
   :lineno-start: 1

The methods that can be implemented are:

* ``initialize``: This method is called once when the controller is initialized. It is used to set the controller parameters.
* ``reset_controller``: This method is called when the controller is reset. It is used to reset the controller internal state (for example the integral term).
* ``set_position``: Set the desired position + yaw and yaw-rate (in deg, deg/s) to track (in the inertial frame in NED).
* ``set_velocity``: Set the desired velocity to track (in the inertial frame in NED).
* ``set_body_velocity``: Set the desired velocity to track (in the body frame in FRD).
* ``set_attitude``: Set the desired attitude (in deg for a FRD body frame relative to a NED inertial frame) and total thrust (in Newton) to track.
* ``set_attitude_rate``: Set the desired angular velocity to track (in deg/s for a FRD body frame relative to a NED inertial frame) and total thrust (in Newton) to track.
* ``set_motor_speed``: Set the individual desired motor speed (0-100%)

Note that you do not need to implement all of these methods, only the ones that are necessary for your controller. **However, keep in mind that
all the autopilot modes can call any of these methods. If an autopilot mode calls a method that is not implemented:**

   1. A runtime exception will be thrown
   2. The autopilot will switch to the failsafe mode (if available)
   3. If the failsafe mode also calls a method that is not implemented, you are screwed.

In practice, most operation modes will call the ``set_position`` method to set the desired position to track. As a good rule of thumb **you should always implement the most generic version of this
method** that receives references up to the snap (even if you do not make use these higher-order derivatives in your controller).
