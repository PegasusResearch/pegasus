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

In this section we detail the mathematical background for the Mellinger controller that is implemented in the Pegasus Autopilot, which sends attitude-rate and total thrust coomands for the inner-loops of the vehicle to track.
This is an adapted version of the algorithm proposed in :cite:p:`Mellinger, Pinto2021`.

Consider the linear motion of the vehicle to be described by Newton's equation of motion

.. math::

   m\dot{v} = mge_3 - TRe_3

Consider the position and velocity trackign error given by

.. math::
   e_p &= p - p_d \\
   \dot{e}_p &= e_v = v - v_d

Consider the total force to be applied to the vehicle body to be given by

.. math::

   F_{des} = -K_p e_p - K_d e_v + mge_3 + m\ddot{p}_d

Next, compute the desired :math:`z_b` axis direction from the desired total force vector

.. math::

   z_b^{des} = \frac{F_{des}}{||F_{des}||}

From the desired yaw angle :math:`\psi_{des}` reference of the body aligned with the inertial frame, we can compute

.. math::

   y_c = \begin{bmatrix} -sin(\psi_{des}) & cos(\psi_{des}) & 0 \end{bmatrix}^\top

The desired :math:`x_b^{des}` axis direction is then given by

.. math::

   x_b^{des} = \frac{y_c^{des} \times z_b^{des}}{||y_c^{des} \times z_b^{des}||}

Finally, the desired :math:`y_b^{des}` axis direction is given by

.. math::

   y_b^{des} = z_b^{des} \times x_b^{des}

The desired attitute of the vehicle can then be encoded in the rotation matrix

.. math::

   R_{des} = \begin{bmatrix} x_b^{des} & y_b^{des} & z_b^{des} \end{bmatrix}  \text{ and } R = \begin{bmatrix} x_b & y_b & z_b \end{bmatrix}

The rotation error can be computed according to

.. math::
   
      e_R = (R_{des}^\top R - R^\top R_{des})^\vee

where :math:`R` is the current rotation matrix of the vehicle. The operator :math:`\vee` is the vee operator that maps a skew-symmetric matrix to a vector and it is defined as

.. math::

   \begin{bmatrix} 0 & -a_3 & a_2 \\ a_3 & 0 & -a_1 \\ -a_2 & a_1 & 0 \end{bmatrix}^\vee = \begin{bmatrix} a_1 \\ a_2 \\ a_3 \end{bmatrix}

To get the desired total thrust to apply to the vehicle, we must project the desired force vector into actual the body z-axis direction (not the desired one that we computed before). This is done by

.. math::

   T = F_{des} \cdot z_b

To compute the desired angular velocity, we must project the jerk expression along the :math:`z_b` axis direction. This is done by


The corresponding code for this controller is implemented in ``pegasus_autopilot/autopilot_controllers/src/mellinger_controller.cpp``. The code is shown below:

.. literalinclude:: ../../../pegasus_autopilot/autopilot_controllers/src/mellinger_controller.cpp
   :language: c++
   :lines: 113-213
   :lineno-start: 1