System Identification
=====================

In this section, we outline the system identification procedure conducted for our multirotor vehicles.

Thrust vs throttle curve identification
----------------------------------

.. note::

   This document is intended for experienced multirotor operators conducting static thrust tests in controlled environments. It assumes familiarity with flight systems, safety protocols, and test equipment.

To control the thrust output of the vehicles, the desired throttle :math:`T_{in} \in [0,1]` must be supplied to the autopilot. Accordingly, a function that relates thrust :math:`T` to throttle :math:`T_{in}` is necessary.

Experimental data points are obtained :math:`(T_{in},T)` by measuring the **static thrust**, defined as the thrust produced when airspeed is zero. These measurements are taken from the multirotor's propellers spinning in free air. 

For accurate data acquisition, a PVC-made vertical pole is mounted on a scale and equipped with discs affixed to its top and bottom. The multirotor is securely attached to the top disc using rope or double-sided adhesive tape. As a general guideline, the pole length is chosen to be at least four times the propeller diameter to minimise ground effect. The scale is positioned on a flat, stable surface to ensure precise thrust measurements, and is calibrated before each measurement session to eliminate drift or zero error.


.. caution:: 

    * Have a clear area around the multirotor.
    * Never approach the aircraft while motors are armed or active.
    * Anchor the scale firmly to a stable surface to avoid vibrations or shifts.
    * Secure the multirotor to prevent unintended movement during operation, especially under high thrust.
    * Ensure your power source is stable and fully charged.
    * Secure loose items, tools, or cabling that may become airborne or interfere with propeller movement.
    * Maintain an active RC link with the multirotor at all times during testing.
    * Have a reliable failsafe mechanism in place, such as a KILL switch.
    * Test the failsafe/KILL switch functionality prior to each session.

.. warning::

   **Command**: KILL the multirotor immediately if any unsafe or undefined test condition arises.

   - Unstable or unexpected flight behaviour.
   - Rotor failure or breakage.
   - Loose or detached propellers.
   - Abnormal or erratic noise from motors or airframe.

    **Neglecting this command may result in injury or equipment damage.**

To prevent attitude stabilisation control from interfering with measurements, the inner-loop autopilot gains are set to zero. The multirotor is then subjected to a range of throttle inputs, and the scale records the thrust produced by the spinning propellers.

To capture the nonlinear relationship between throttle input and thrust output, the following model is employed,

.. math::

    T = a \arctan\left(b T_{in} + c\right) + d.

This formulation supports straightforward analytical inversion, which is useful for multirotor control. The parameters :math:`a`, `b`, `c`, and `d` are determined through curve fitting techniques applied to the experimental dataset :cite:p:`Pinto2025`.

**System Identification Video**
TODO

..  youtube:: dEQMgvO_WxI
    :width: 100%
    :align: center
    :privacy_mode: