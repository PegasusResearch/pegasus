
Mellinger Controller
====================

In this section we detail the mathematical background for the Mellinger controller that is implemented in the Pegasus Autopilot. It sends angular rates and total thrust commands to the inner-loops of the vehicle to be tracked.
This is an adapted version of the algorithm proposed in :cite:p:`Mellinger, Pinto2021`. Reference trajectories :math:`\{p_{des},~v_{des},~a_{des},~j_{des}\}` should be three times continuously differentiable.

Mathematical Background 
-----------------------

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

Implementation
--------------

This controller is implemented in ``pegasus_autopilot/autopilot_controllers/src/mellinger_controller.cpp``. The code is shown below:

.. literalinclude:: ../../../../../pegasus_autopilot/autopilot_controllers/src/mellinger_controller.cpp
   :language: c++
   :lines: 113-201
   :lineno-start: 1