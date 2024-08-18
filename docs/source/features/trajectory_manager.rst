Trajectory Manager
==================

In this section, we explain how the define parametric paths or trajectories to be followed by the vehicle.

0. Mathematical Background
--------------------------

Consider an arbitrary path/trajectory parameterized by a parameter :math:`\gamma(t)` given by the Trajectory Manager. Consider the desired position to be given by :math:`p_d(\gamma) \in \mathbb{R}^3`, such that:

1. The desired velocity to be tracked in the inertial frame is given by

   .. math:: \dot{p}_d(\gamma) = \frac{\partial p_d(\gamma)}{\partial \gamma}\dot{\gamma}

2. The desired acceleration to be tracked in the inertial frame is given by

   .. math:: 
      
      \ddot{p}_d(\gamma) &=\frac{d}{dt} \left(\frac{\partial p_d(\gamma)}{\partial \gamma}\dot{\gamma}\right) \\
                         &= \frac{d}{dt} \left(\frac{\partial p_d(\gamma)}{\partial \gamma}\right)\dot{\gamma} + \frac{\partial p_d(\gamma)}{\partial \gamma} \ddot{\gamma}\\
                         &= \frac{\partial^2 p_d(\gamma)}{\partial \gamma^2}\dot{\gamma}^2 + \frac{\partial p_d(\gamma)}{\partial \gamma} \ddot{\gamma}


3. The desired jerk to be tracked in the inertial frame is given by

   .. math:: 
      
      \dddot{p}_d(\gamma) &= \frac{d}{dt}\left(\frac{\partial^2 p_d(\gamma)}{\partial \gamma^2}\dot{\gamma}^2 + \frac{\partial p_d(\gamma)}{\partial \gamma} \ddot{\gamma}\right) \\
                          &=  \frac{d}{dt}\left(\frac{\partial^2 p_d(\gamma)}{\partial \gamma^2}\dot{\gamma}^2\right) + \frac{d}{dt}\left(\frac{\partial p_d(\gamma)}{\partial \gamma} \ddot{\gamma}\right) \\
                          &= \frac{\partial^3 p_d(\gamma)}{\partial \gamma^3}\dot{\gamma}^3 + 2\frac{\partial^2 p_d(\gamma)}{\partial \gamma^2}\dot{\gamma}\ddot{\gamma} + 
                          \frac{\partial^2 p_d(\gamma)}{\partial \gamma^2} \dot{\gamma}\ddot{\gamma} + \frac{\partial p_d(\gamma)}{\partial \gamma} \dddot{\gamma} \\
                          &= \frac{\partial^3 p_d(\gamma)}{\partial \gamma^3}\dot{\gamma}^3 + 3\frac{\partial^2 p_d(\gamma)}{\partial \gamma^2}\dot{\gamma}\ddot{\gamma} + \frac{\partial p_d(\gamma)}{\partial \gamma} \dddot{\gamma}

1. Definition of the Trajectory Interface
-----------------------------------------
.. literalinclude:: ../../../pegasus_autopilot/static_trajectory_manager/include/static_trajectory_manager/static_trajectory.hpp
   :language: c++
   :emphasize-lines: 67-73
   :lines: 53-180
   :linenos:


2. Provided Trajectories
------------------------
TODO

3. Adding Custom Static Trajectories
------------------------------------
TODO
