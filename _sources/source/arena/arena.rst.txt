Taguspark Flight Arena
======================

The Taguspark flight arena is an indoor space, under the administration of the ISR-Lisbon, which features an OptiTrack Motion Capture System (MCS).
The MCS consists of 8 cameras that are mounted on the ceiling of the arena and is used to track the position and orientation of rigid bodies equipped with reflective markers.
A dedicated computer streams the obtained data into a computer network.
The arena dimensions are 7.0 x 4.0 x 2.5 m.

.. image:: /_static/taguspark_arena/arena_pictures/taguspark_arena.jpg
        :width: 600px
        :align: center
        :alt: Taguspark flight arena

|

Here we describe the setup and calibration procedures for the OptiTrack MCS.

Setting up Optitrack with Motive
--------------------------------

0. Turn on the OptiTrack cameras, the arena computer and router. Get also the calibration tools ready.

1. Launch the Motive application from the Windows desktop.

    .. image:: /_static/taguspark_arena/calibration/01_Motive.jpg
            :width: 600px
            :align: center
            :alt: Motive application
            
.. |calibration_pane_icon| image:: /_static/taguspark_arena/calibration/02_calibration_pane_icon.png

2. Open the Camera Calibration pane |calibration_pane_icon| and click on ``Clear Mask`` followed by ``Mask Visible`` to mask all the bright spots in the arena that can be mistakenly identified as passive markers. 

    .. image:: /_static/taguspark_arena/calibration/02_mask_visible.png
            :width: 600px
            :align: center
            :alt: Mask visible reflective regions

3. Start the calibration process by clicking on ``Start Wanding``.

    .. image:: /_static/taguspark_arena/calibration/03_start_wanding.png
            :width: 600px
            :align: center
            :alt: Start wanding

4. Enter the arena with the **calibration wand** in hand and wave it around the arena. Make sure that the LED indicator ring of all cameras is completely filled in green.

    .. image:: /_static/taguspark_arena/calibration/04_led_indicator_ring.jpg
            :width: 600px
            :align: center
            :alt: Led indicator ring

5. Click on ``Calculate`` followed by ``Apply`` and wait for the `Calibration Result Report` window to pop up.

    .. image:: /_static/taguspark_arena/calibration/05_calibration_done.png
            :width: 600px
            :align: center
            :alt: Performing the camera calibration

6. Check the `Calibration Result Report`. If it reports an **Excellent** or **Exceptional** calibration result, press ``Apply``. Else, press ``Cancel`` and repeat the wanding process.

    .. image:: /_static/taguspark_arena/calibration/06_calibration_result_report.png
            :width: 600px
            :align: center
            :alt: Calibration result report

7. Place the **calibration square** in the center of the arena, with the shorter leg pointing towards the windows.

    .. image:: /_static/taguspark_arena/calibration/07_ground_plane_standard.jpg
            :width: 600px
            :align: center
            :alt: Ground plane standard

.. admonition:: \ \ 

    Have in mind that the motion data captured from the MCS is expressed in an East North Up (ENU) inertial frame, which is the standard in the ROS representation.
    The `mavlink interface` converts this data to a North East Down (NED) inertial frame with the attitude following the Front Right Down (FRD) of the rigid body.

    .. Defining the standard orientation received in ROS 2. The data is received in the ENU format, with the attitude following the FLU of the body with respect to the ENU inertial, expressed in the ENU inertial frame.
    .. This data is converted in the mavlink interface to the NED format with the attitude following the FRD of the body with respect to the NED inertial, expressed in the NED inertial frame.


8. Open the `Ground Plane` tab of the Camera Calibration pane and click on ``Set Ground Plane`` to finish the calibration process.

    .. image:: /_static/taguspark_arena/calibration/08_set_ground_plane.png
            :width: 600px
            :align: center
            :alt: Set the ground plane

.. admonition:: \ \ 

    You can save your calibration file for later use. It can be used during multiple consective days of experiments in the arena.
    However, the calibration accuracy naturally deteriorates over time due to ambient factors, such as fluctuations in temperature. 
    **You should re-calibrate the MCS on a weekly basis**.

9. Take the calibration square out of the arena and place your vehicle inside the arena **with its front pointing towards the windows**. Select the group of markers you see on screen at once and check if all the markers placed on your vehicle are detected.

    .. image:: /_static/taguspark_arena/calibration/09_selecting_the_vehicle.png
            :width: 600px
            :align: center
            :alt: Selecting the markers that will represent the vehicle

.. |assets_pane_icon| image:: /_static/taguspark_arena/calibration/10_assets_pane_icon.png

10. Create a rigid body from the selected markers by right-clicking over them and choosing the ``Rigid Body`` followed by ``Create From Selected Markers``. On the Assets pane |assets_pane_icon|, you should see the newly created rigid body.

    .. image:: /_static/taguspark_arena/calibration/10_creating_the_rigid_body.png
            :width: 600px
            :align: center
            :alt: Creating the rigid body from the marker selection