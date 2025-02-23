Controllers
===========

A ``controller`` computes the control inputs for the vehicle to track a desired trajectory given an estimated state of the vehicle. 
The controller can be used by all the autopilot ``modes`` to compute the control inputs for the vehicle. 

If you want to prototype a new controller to be used only for following
a specific trajectory, then it might make sense to implement it as a ``mode`` rather than a ``controller`` that is used by all the other available modes.

This page is structured as follows:

* Section :ref:`Controller Interface` details the interface that a controller must implement to be used by the autopilot.
* Section :ref:`PID Controller` details the mathematical background for the PID controller that is implemented in the Pegasus Autopilot.
* Section :ref:`Mellinger Controller` details the mathematical background for the Mellinger controller that is implemented in the Pegasus Autopilot.
* Section :ref:`Adding a Custom Controller` details how to add a custom controller to the autopilot.

**Controllers Page Index:**

.. toctree::
   :glob:
   :maxdepth: 4

   interface
   pid
   mellinger
   custom_controller
