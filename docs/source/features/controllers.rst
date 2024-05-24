Controllers
===========

0. Definition of the Controller Interface
-----------------------------------------
.. literalinclude:: ../../../pegasus_autopilot/autopilot/include/autopilot/controller.hpp
   :language: c++
   :emphasize-lines: 31-34, 36-40, 42-49, 51-53, 55-57, 59-61, 63-65, 67-69, 71-78, 80-87, 89-96, 98-105, 107-112
   :lines: 62-186
   :lineno-start: 1

1. PID Controller - Mathematical Background
-------------------------------------------

In this section we detail the mathematical background for the PID controller that is implemented in the Pegasus Autopilot, which sends attitude and total thrust commands for the inner-loops of the vehicle to track.
This is the same algorithm used in :cite:p:`Jacinto2021mastersthesis,jacinto2022chemical`.

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

   u &= - \frac{T}{m}R e_3 \\
     &= - \frac{T}{m}R_z({\psi_{des}})R_y({\theta_{des}})R_x({\phi_{des}}) e_3 \\
     & = - \frac{T}{m}R_z({\psi_{des}})r_{3d} \\

we can take the total thrust (in Newton) to be given by :math:`T = m|| u ||`. Replacing in the previous equations yields

.. math::

   r3d = -R_z^\top({\psi_{des}})\frac{u}{||u||}

But we also know that 

.. math::

   r3d = R_y({\theta_{des}})R_x({\phi_{des}}) e_3 = \begin{bmatrix} cos(\phi_{des})sin(\theta_{des}) \\ -sin(\phi_{des}) \\ cos(\theta_{des})cos(\phi_{des}) \end{bmatrix} = \begin{bmatrix} r_{31} \\ r_{32} \\ r_{33} \end{bmatrix}

.. admonition:: Reference Frames

   Reminder that the reference frames are defined as follows:
   - The inertial frame :math:`\mathcal{I}` is defined according to the North-East-Down (NED) convention.
   - The body frame: :math:`\mathcal{B}` is defined according to the Forward-Right-Down (FRD) convention.

   Therefore, a minus sign appears in the :math:`r_{3d}` as the body z-axis is pointing downwards, but the acceleration vector is pointing upwards.

Finally, we can compute the desired roll :math:`\phi_{des}`, pitch :math:`\theta_{des}` angles as follows

.. math::

   \phi_{des} &= arcsin(-r_{32}) \\
   \theta_{des} &= arctan \left(\frac{r_{31}}{r_{33}}\right)

The corresponding code for computing the desired acceleration is implemented in ``pegasus_autopilot/autopilot_controllers/src/pid_controller.cpp``. The code is shown below:

.. literalinclude:: ../../../pegasus_autopilot/autopilot_controllers/src/pid_controller.cpp
   :language: c++
   :lines: 109-145
   :lineno-start: 1

The code for converting the desired acceleration into a set of desired roll and pitch angles + total thrust is implemented in
``pegasus_addons/thrust_curves/include/thrust_curves/acceleration_to_attitude.hpp``. The code is shown below:

.. literalinclude:: ../../../pegasus_addons/thrust_curves/include/thrust_curves/acceleration_to_attitude.hpp
   :language: c++
   :lines: 64-85
   :lineno-start: 1

.. admonition:: Integral Action

   Since the quadrotor can be seen as a double integrator, the integral action is not necessary for the system to be stable, and the natural position controller that emerges is a Proportional-Derivative (PD). However, in practice a little bit of integral action can be used to improve the tracking performance of the system, to accomodade for model uncertainties (such as the mass).

2. Mellinger Controller - Mathematical Background
-------------------------------------------------