Modes
======

An operation ``mode`` is a state in which the autopilot can be in. The autopilot can be in only one mode at a time.
In this section we provide an overview on the autopilot operating modes and how to add custom modes to the state machine.

Depending on the target application, an autopilot ``mode`` can be used to define a new behaviour for the vehicle to execute,
or be used to prototype a controller as well. For instance, if you want to prototype a new controller to be used only for following
a specific trajectory, then it might make sense to implement it as a ``mode`` rather than a ``controller`` that is used by all the other available modes.

This page is structured as follows:

* Section :ref:`Mode Interface` introduces the "Mode" class that defines the interface for the autopilot operating modes.
* Section :ref:`Provided State Machine Modes` provides an overview of the provided operating modes and where they are implemented.
* Section :ref:`Adding Custom Modes to the State Machine` provides a step-by-step guide on how to add custom operating modes to the state machine.

**Operation Modes Page Index:**

.. toctree::
   :glob:
   :maxdepth: 4

   interface
   provided_modes
   custom_modes