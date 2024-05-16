Taguspark Flight Arena
======================


Setting up Optitrack with Motive
--------------------------------

0. Turn the arena computer and router

1. TODO ...

    .. image:: /_static/taguspark_arena/calibration/0_Motive.jpg
            :width: 600px
            :align: center
            :alt: Motive application

2. TODO ...

    .. image:: /_static/taguspark_arena/calibration/1_mask_visible.png
            :width: 600px
            :align: center
            :alt: Mask visible reflective regions

3. TODO ...

    .. image:: /_static/taguspark_arena/calibration/2_start_wanding.png
            :width: 600px
            :align: center
            :alt: Start wanding

4. TODO ...

    .. image:: /_static/taguspark_arena/calibration/3_calibration_done.png
            :width: 600px
            :align: center
            :alt: Performing the camera calibration

5. TODO ...

    .. image:: /_static/taguspark_arena/calibration/4_set_ground_plane.png
            :width: 600px
            :align: center
            :alt: Set the ground plane


Defining the standard orientation received in ROS 2. The data is received in the ENU format, with the attitude following the FLU of the body with respect to the ENU inertial, expressed in the ENU inertial frame.

This data is converted in the mavlink interface to the NED format with the attitude following the FRD of the body with respect to the NED inertial, expressed in the NED inertial frame.
    .. image:: /_static/taguspark_arena/calibration/4_1_ground_plane_standard.jpg
            :width: 600px
            :align: center
            :alt: Ground plane standard

6. TODO ...

    .. image:: /_static/taguspark_arena/calibration/5_selecting_the_vehicle.png
            :width: 600px
            :align: center
            :alt: Selecting the markers that will represent the vehicle

7. TODO ...

    .. image:: /_static/taguspark_arena/calibration/6_creating_the_rigid_body.png
            :width: 600px
            :align: center
            :alt: Creating the rigid body from the marker selection