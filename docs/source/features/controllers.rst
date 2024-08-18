Controllers
===========

A ``controller`` computes the control inputs for the vehicle to track a desired trajectory given an estimated state of the vehicle. 
The controller can be used by all the autopilot ``modes`` to compute the control inputs for the vehicle. 

If you want to prototype a new controller to be used only for following
a specific trajectory, then it might make sense to implement it as a ``mode`` rather than a ``controller`` that is used by all the other available modes.

This page is structured as follows:

* Section :ref:`0. Controller Interface` details the interface that a controller must implement to be used by the autopilot.
* Section :ref:`1. PID Controller - Mathematical Background and Implementation` details the mathematical background for the PID controller that is implemented in the Pegasus Autopilot.
* Section :ref:`2. Mellinger Controller - Mathematical Background and Implementation` details the mathematical background for the Mellinger controller that is implemented in the Pegasus Autopilot.
* Section :ref:`3. Adding a Custom Controller` details how to add a custom controller to the autopilot.

0. Controller Interface
-----------------------

The interface for the autopilot controller is defined in the autopilot package under the ``pegasus_autopilot/autopilot/include/autopilot/controller.hpp`` header file. 
The ``Controller`` class is an abstract class that defines the interface for the autopilot controller.

Each autopilot controller **must** inherit from the ``Controller`` class and **implement the following one or multiple methods highlighted in yellow** in the code snippet below:

.. literalinclude:: ../../../pegasus_autopilot/autopilot/include/autopilot/controller.hpp
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


1. PID Controller - Mathematical Background and Implementation
--------------------------------------------------------------

In this section we detail the mathematical background for the PID controller that is implemented in the Pegasus Autopilot, which sends attitude and total thrust commands for the inner-loops of the vehicle to track.
This is the same algorithm used in :cite:p:`Jacinto2021mastersthesis,jacinto2022chemical`.

Consider a multirotor vehicle given by the double-integrator model:

.. math::

   \ddot{p} &= g e_3 \underbrace{- \frac{T}{m}R e_3}_{u} \\
            &= g e_3 + u

where :math:`u \in \mathbb{R}^3` is the acceleration input of the system. Consider also the tracking error system:

.. math::

   e_p &= p_d - p \\
   \dot{e}_p &= e_v = v_d - v 

The controller can then be given by the Proportional-Derivative (PD) controller with an acceleration feed-forward term

.. math::

   u = K_p e_p + K_d e_v + a_d - ge_3,

can render the system globally assymptotically stable (GAS). From the equality

.. math::

   u &= - \frac{T}{m}R e_3 \\
     &= - \frac{T}{m}R_z({\psi_{des}})R_y({\theta_{des}})R_x({\phi_{des}}) e_3 \\
     & = - \frac{T}{m}R_z({\psi_{des}})r_{3d} \\

we can take the total thrust (in Newton) to be given by :math:`T = m|| u ||`. Replacing in the previous equations yields

.. math::

   r_{3d} = -R_z^\top({\psi_{des}})\frac{u}{||u||}

But we also know that 

.. math::

   r_{3d} = R_y({\theta_{des}})R_x({\phi_{des}}) e_3 = \begin{bmatrix} \cos(\phi_{des})\sin(\theta_{des}) \\ -\sin(\phi_{des}) \\ \cos(\theta_{des})\cos(\phi_{des}) \end{bmatrix} = \begin{bmatrix} r_{31} \\ r_{32} \\ r_{33} \end{bmatrix}

.. admonition:: Reference Frames

   Reminder that the reference frames are defined as follows:
   - The inertial frame :math:`\mathcal{I}` is defined according to the North-East-Down (NED) convention.
   - The body frame: :math:`\mathcal{B}` is defined according to the Forward-Right-Down (FRD) convention.

   Therefore, a minus sign appears in the :math:`r_{3d}` as the body z-axis is pointing downwards, but the acceleration vector is pointing upwards.

Finally, we can compute the desired roll :math:`\phi_{des}`, pitch :math:`\theta_{des}` angles as follows

.. math::

   \phi_{des} &= \arcsin(-r_{32}) \\
   \theta_{des} &= \arctan \left(\frac{r_{31}}{r_{33}}\right)

The corresponding code for computing the desired acceleration is implemented in ``pegasus_autopilot/autopilot_controllers/src/pid_controller.cpp``. The code is shown below:

.. literalinclude:: ../../../pegasus_autopilot/autopilot_controllers/src/pid_controller.cpp
   :language: c++
   :lines: 105-141
   :lineno-start: 1

The code for converting the desired acceleration into a set of desired roll and pitch angles + total thrust is shown below:

.. literalinclude:: ../../../pegasus_autopilot/autopilot_controllers/src/pid_controller.cpp
   :language: c++
   :lines: 155-175
   :lineno-start: 1

.. admonition:: Integral Action

   Since the quadrotor can be viewed as a double integrator, the integral action is not necessary for the system to be stable, and the natural position controller that emerges is a Proportional-Derivative (PD). However, in practice a little bit of integral action can be used to improve the tracking performance of the system and accomodade for model uncertainties (such as the mass).

2. Mellinger Controller - Mathematical Background and Implementation
--------------------------------------------------------------------

In this section we detail the mathematical background for the Mellinger controller that is implemented in the Pegasus Autopilot. It sends angular rates and total thrust commands to the inner-loops of the vehicle to be tracked.
This is an adapted version of the algorithm proposed in :cite:p:`Mellinger, Pinto2021`. Reference trajectories :math:`\{p_{des},~v_{des},~a_{des},~j_{des}\}` should be three times continuously differentiable.

The vehicle translational dynamics derive from Newton's 2nd law of motion and are expressed as

.. math::

   m\dot{v} = mge_3 - TRe_3

where :math:`R = \left[ \begin{matrix} x_b & y_b & z_b \end{matrix} \right]` denotes the vehicle orientation. 
Define the position and velocity tracking error as

.. math::

   e_p &= p_{des} - p \\
   \dot{e}_p &= e_v = v_{des} - v 

The total force to be applied to the vehicle body is given by

.. math::

   F_{des} = m (K_p e_p + K_d e_v - ge_3 + a_{des})

Next, compute the desired :math:`z_b` axis from the desired total force

.. math::

   z_b^{des} = -\frac{F_{des}}{||F_{des}||}

.. admonition:: Reference Frames

   The reference frames are defined as follows:
   - The inertial frame :math:`\mathcal{I}` is defined according to the North-East-Down (NED) convention.
   - The body frame: :math:`\mathcal{B}` is defined according to the Forward-Right-Down (FRD) convention.

   As the body z-axis is pointing downwards, but the force vector is pointing upwards, a minus sign appears in the expression of :math:`z_b^{des}`.

Get the desired total thrust to apply to the vehicle by projecting :math:`F_{des}` onto the actual body z-axis (not the desired one that we computed before),

.. math::

   T = -F_{des}^\top z_b

.. From the desired yaw angle :math:`\psi_{des}` reference of the body aligned with the inertial frame, we define

The orientation kinematics are governed by the following equation

.. math::

   \dot{R} = R (\omega)_{\times}

where :math:`(\omega)_{\times}` is the skew-symmetric matrix of the angular velocity vector.
To drive the vehicle orientation to a desired reference :math:`R_{des}`, we design a control law that provides angular rate setpoints. It is defined as

.. math::
   
   \omega =  -K_R e_R + \omega_{des}

where the rotation error :math:`e_R` is computed according to

.. math::
   
      e_R = \frac{1}{2}(R_{des}^\top R - R^\top R_{des})^\vee

The operator :math:`\vee` maps a skew-symmetric matrix to a vector as follows

.. math::

   \begin{bmatrix} 0 & -a_3 & a_2 \\ a_3 & 0 & -a_1 \\ -a_2 & a_1 & 0 \end{bmatrix}^\vee = \begin{bmatrix} a_1 \\ a_2 \\ a_3 \end{bmatrix}


We now present the steps to compute :math:`R_{des}` and :math:`\omega_{des}`. 
To obtain :math:`R_{des} = \left[ \begin{matrix} x_b^{des} & y_b^{des} & z_b^{des} \end{matrix} \right]`, start by defining

.. math::

   y_c = \begin{bmatrix} -\sin(\psi_{des}) & \cos(\psi_{des}) & 0 \end{bmatrix}^\top

from the desired yaw angle :math:`\psi_{des}`. With this definition, the desired :math:`x_b^{des}` and :math:`y_b^{des}` axes are given by simple expressions,

.. math::

   x_b^{des} &= \frac{y_c^{des} \times z_b^{des}}{||y_c^{des} \times z_b^{des}||} \\
   y_b^{des} &= z_b^{des} \times x_b^{des}

As for :math:`\omega_{des} = \left[ \begin{matrix} p_{des} & q_{des} & r_{des} \end{matrix} \right]^\top`, let :math:`j` denote the jerk of the vehicle and take the first time-derivative of the translational dynamics to get

.. math::

      \frac{d}{dt}(ma) &= \frac{d}{dt}(mge_3) - \frac{d}{dt}(TRe_3) \\
      \Leftrightarrow m j &= -\dot{T}Re_3 - T\frac{d}{dt}\left(Re_3 \right) \\
                               &= -\dot{T}Re_3 - TR\omega \times Re_3 \\
                               &= -\dot{T}z_b - TR \left( \omega \times e_3 \right) \\
                               &= -\dot{T}z_b + T R \left(e_3 \times \omega \right)  \\
                               &= -\dot{T}z_b + T (-q x_b + p y_b) 

.. &= -\dot{T}z_b + T R \begin{bmatrix} -q \\ p \\ 0\end{bmatrix} \\

* If we left multiply the jerk equation by :math:`x_b^{\top}` we get

.. math::

   m x_b^{\top}j &= \underbrace{-\dot{T}x_b^{\top}z_b}_{0} + T (\underbrace{-q x_b^\top x_b}_{-q} + \underbrace{p y_b^\top x_b}_{0}) \\
   \Leftrightarrow q &= -\frac{m}{T}x_b^{\top}j

* If we left multiply the jerk equation by :math:`y_b^{\top}` we get

.. math::

   m y_b^{\top}j &= \underbrace{-\dot{T}y_b^{\top}z_b}_{0} + T (\underbrace{-q x_b^\top y_b}_{0} + \underbrace{p y_b^\top y_b}_{p}) \\
   \Leftrightarrow p &= \frac{m}{T}y_b^{\top}j

* The angular velocity about the z-axis can be obtained from the first time derivative of the yaw angle

.. math::

   r = \dot{\psi} e_3^\top z_b

Then given orientation :math:`R_{des}` and jerk :math:`j_{des}` references, the desired angular rates are given by

.. math::

   \omega_{des} = \begin{bmatrix} \frac{m}{T}{y^{des}_b}^{\top}j^{des} \\ -\frac{m}{T}{x^{des}_b}^{\top}j^{des} \\ \dot{\psi}_{des} e_3^\top z_b^{des} \end{bmatrix}

.. .. admonition:: Feed-forward references

..    Note that we have expressed :math:`p`, :math:`q` and :math:`r` angular-velocities as a function of :math:`x_b`, :math:`y_b` and :math:`z_b` axis, but our goal is to get the desired feed-forward
..    terms for the angular velocities to feed to a proportional controller. Therefore, we must compute :math:`p_{des}`, :math:`q_{des}` and :math:`r_{des}` using :math:`x_b^{des}`, :math:`y_b^{des}` and :math:`z_b^{des}`.

This controller is implemented in ``pegasus_autopilot/autopilot_controllers/src/mellinger_controller.cpp``. The code is shown below:

.. literalinclude:: ../../../pegasus_autopilot/autopilot_controllers/src/mellinger_controller.cpp
   :language: c++
   :lines: 113-201
   :lineno-start: 1

3. Adding a Custom Controller
------------------------------

TODO
