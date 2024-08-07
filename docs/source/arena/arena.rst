Taguspark Flight Arena
======================

The Taguspark flight arena is an indoor space, under the administration of the ISR-Lisbon, which features an OptiTrack Motion Capture System (MCS).
The MCS consists of 8 cameras that are mounted on the ceiling of the arena and is used to track the position and orientation of rigid bodies equipped with reflective markers.
A dedicated computer streams the obtained data into a computer network.
The arena dimensions are 7.8 x 5.0 x 2.5 m.

.. image:: /_static/taguspark_arena/arena_pictures/taguspark_arena.jpg
        :width: 600px
        :align: center
        :alt: Taguspark flight arena

.. list-table:: Arena Network Configurations
   :widths: 5 15 
   :header-rows: 0
    
   * - Station SSID
     - Quadrotor
   * - Station Password
     - 
   * - IP Range
     - 192.168.1.0/24
   * - IPs reserved for vehicles
     - 192.168.1.241 to 192.168.1.254
   * - Desktop IP
     - 192.168.1.100
   * - Free IP for laptop
     - 192.168.1.240
      
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


Troubleshooting
-----------------

Here we provide some insight to solve some of the most frequent issues.
Please check the `Motive reference page <https://docs.optitrack.com/motive/rigid-body-tracking>`_ for more information on the calibration and troubleshooting procedures.

Shaky Rigid Body Asset
^^^^^^^^^^^^^^^^^^^^^^

If by looking at an asset in the 3D viewport you notice that it is shaking when the Rigid body is stationary, you should then resort to the Rigid Body refinement tool to improve the accuracy of the Rigid Body calculations in Motive.
This tool allows Motive to collect additional samples, achieving more accurate tracking results by improving the calculation of expected marker locations of the Rigid Body as well as the position and orientation of the Rigid Body itself.

.. |builder_pane_icon| image:: /_static/taguspark_arena/calibration/builder_pane_icon.png

1. From the View menu, open the Builder pane, or click the |builder_pane_icon| button on the toolbar.

    .. image:: /_static/taguspark_arena/calibration/open_builder_pane.png
            :width: 600px
            :align: center
            :alt: Open the Builder pane

2. Click the Edit tab.

3. Select the Rigid Body to be refined in the Assets pane.

4. Place the Rigid Body in the center of the arena so as many cameras as possible can clearly capture its markers.

  a. In the **Refine** section of the Edit tab of the Builder pane, click *Start...*

  b. **Slowly rotate** the Rigid Body to collect samples at different orientations until the progress bar is full.

 .. image:: /_static/taguspark_arena/calibration/builder_pane_refine.png
            :width: 600px
            :align: center
            :alt: Refine asset


Marker Placement
^^^^^^^^^^^^^^^^^

* Placing too many markers on one Rigid Body is not recommended.
  When too many markers are placed in close vicinity, markers may overlap on the camera view, and Motive may not resolve individual reflections. This can increase the likelihood of label-swaps during capture.

  .. admonition:: \ \
    
    Whenever possible, though, it is best to use 4 or more markers to create a Rigid Body. 
    Additional markers provide more 3D coordinates for computing positions and orientations of a rigid body, making overall tracking more stable and less vulnerable to marker occlusions.

* Placing the markers in symmetrical shapes such as squares, isosceles, or equilateral triangles make asset identification difficult and may cause the Rigid Body assets to flip during capture.
  Within a Rigid Body asset, the markers should be placed asymmetrically because this provides a clear distinction of orientations.

* Not having unique Rigid Bodies could lead to labeling errors especially when tracking several assets with similar size and shape. 
  When tracking multiple objects using passive markers, it is beneficial to create unique Rigid Body assets in Motive. Specifically, you need to place retroreflective markers in a distinctive arrangement between each object, which will allow Motive to more clearly identify the markers on each Rigid Body throughout capture.