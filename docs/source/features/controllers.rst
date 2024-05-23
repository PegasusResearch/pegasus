Controllers
===========

0. Definition of the Controller Interface
-----------------------------------------
.. literalinclude:: ../../../pegasus_autopilot/autopilot/include/autopilot/controller.hpp
   :language: c++
   :emphasize-lines: 31-34, 36-40, 42-49, 51-53, 55-57, 59-61, 63-65, 67-69, 71-78, 80-87, 89-96, 98-105, 107-112
   :lines: 62-186
   :lineno-start: 1

1. Mathematical Background
--------------------------

* PID controller

Consider a multirotor vehicle given by the double-integrator model:

.. math::

   \ddot{p} &= g e_3 \underbrace{- \frac{T}{m}R e_3}_{u} \\
            &= g e_3 + u

where :math:`u \in \mathbb{R}^3` is the acceleration input of the system. Consider also the tracking error system:

.. math::

   e_p &= p - p_d \\
   \dot{e}_p &= e_v = v - v_d

The controller can then be given by the Proportional-Derivative (PD) controller with an acceleration feed-forward term

.. math::

   u = -K_p e_p -K_d e_v + a_d -ge_3,

can render the system globally assymptotically stable (GAS). From the equality

.. math::

   u = - \frac{T}{m}R e_3

we can take the total thrust (in Newton) to be given by :math:`T = || u ||`` and the body z-axis of the vehicle
to be given by :math:`z_b = R e_3 = \frac{u}{||u||}`.