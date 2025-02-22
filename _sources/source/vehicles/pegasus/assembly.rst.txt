Drone Assembly
==============

.. image:: /_static/vehicles/pegasus/carbon_fiber_dimensions.png
  :width: 800
  :align: center
  :alt: Pegasus top carbon-fiber sheet view

.. raw:: html

  <div style="margin-bottom: 20px;"></div>

Bill of Materials
-----------------

In order to replicate your Pegasus prototype, you will need some electronic components and mad soldering skills 👨🏻‍🔧. For the onboard PC (where the Pegasus GNC code will run) you will need:

* 1x `Nvidia Jetson Orin Nano Developer Kit <https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit>`_
* 1x `1Tb NVMe SSD PCIe 4.0 <https://www.google.com/search?q=1Tb%20NVMe%20SSD%20PCIe%204.0>`_
* 1x `Realsense d435i <https://www.intelrealsense.com/depth-camera-d435i/>`_
* 1x `USB 3.1 Type A to Type C <https://www.google.com/search?q=USB+3.0+Type+A+to+Type+C+cable>`_

.. admonition:: Note

  The USB cable with 0.30m is used to connect the Realsense camera. The cable should support high-speed transfers and have good shielding, otherwise we might get GPS interference. This length should be enough to connect the camera to the Jetson Orin Nano without any extra slack.

We also make use of integrated FPV board, because they already provide drivers for the most common sensors and are easy to setup. In this case we chose the Kakute H7.mini v1.3 (or the Kakute H7 regular), which includes a microcontroller that can run `PX4 <https://docs.px4.io/main/en/>`_ and a nice 4in1 ESC. The motors and propellers are also important components for the drone.

* 1x `Kakute H7-mini v1.3 microcontroller <https://holybro.com/products/kakute-h7-mini>`_ 
* 1x `4in1 ESC 65A <https://holybro.com/collections/autopilot-peripherals/products/tekko32-f4-metal-4in1-65a-esc-65a>`_
* 4x `T-motor V2306 V2 2400KV <https://www.google.com/search?q=t-motor+V2306+V2+2400KV>`_
* 1x `Micro M8N GPS <https://holybro.com/collections/standard-gps-module/products/micro-m8n-gps>`_
* 4x `Propellers Dalprop Cyclone T5050 <https://www.google.com/search?q=dalprop+cyclone+5050>`_
* 1x `Battery 4S 4300mAh <https://www.google.com/search?q=Battery+4S+4300mAh>`_
* 1x `Set of jumper dupont wires <https://www.google.com/search?q=jumper+dupont+wires>`_
* 1x `XT60 cable connector <https://www.google.com/search?q=XT60+cable+connector>`_
* 15x `50mm Aluminum M3 standoffs spacers <https://www.google.com/search?q=50mm+Aluminum+M3+standoffs+spacers>`_

You will also need:

* 2x carbon-fiber sheets (custom cut)
* 3D printed parts to build the frame of the drone. 

The carbon-fiber sheets should be cut to the dimensions of the drone frame. Check the :ref:`CAD Models` section for more information.

.. admonition:: Note
  
  You can choose batteries with higher capacity than 4300 mAh, but this will tipically increase the weight of the drone. In addition the operating voltage of a 4S battery (12-16.8V) should be whithin the range of operation of the standard `Jetson Orin DC-jack carrier board input (9-20V) <https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwiOyP6s4deLAxWxQ6QEHS8yHaYQFnoECBcQAQ&url=https%3A%2F%2Fdeveloper.nvidia.com%2Fdownloads%2Fassets%2Fembedded%2Fsecure%2Fjetson%2Forin_nano%2Fdocs%2Fjetson_orin_nano_devkit_carrier_board_specification_sp.pdf&usg=AOvVaw0kftK-aC9T6YmeII0KLDui&opi=89978449>`_, hence it is a good choice for this setup as it does not require extra electronics for voltage conversion (as long as you have a good capacitor soldered in parallel).

You might want to consider buying an altimeter and an optical flow sensor to increase the drone's stability and precision (if compatible with your selected board). The recommended sensors are:

* 1x `VL53L1X Lidar from Holybro <https://holybro.com/products/st-vl53l1x-lidar>`_ (works up to 4m)
* 1x `PMW3901 Optical Flow Sensor <https://holybro.com/products/pmw3901-optical-flow-sensor>`_

The remote control and receiver are used to control the drone:

* 1x `FrSky R-XSR ACCST D16 <https://www.frsky-rc.com/product/r-xsr/>`_
* 1x `Taranix X9 Lite RC remote <https://www.frsky-rc.com/product/taranis-x9-lite>`_

You can choose other remote control and receiver combinations as long as they are compatible with the Kakute H7 mini stack. An example alternative is:

* 1x `Spektrum DSMX RC remote <https://www.spektrumrc.com/aircraft-transmitters/>`_
* 1x `Spektrum receiver <https://www.spektrumrc.com/aircraft-receivers/>`_

.. admonition:: Extra - Wifi Antennas

  You might want to consider buying an additional set of antennas for the Jetson Orin Nano carrier board with 8dBi gain, to increase the range of the drone. This is particularly useful if you are planning to fly the drone outdoors. The connector compatible with the NVIDIA carrier board is of the type **U.FL IPX IPEX MHF4 for RP-SMA**.

Extra tools and materials 🛠️:

* 1x `Soldering iron/station <https://www.google.com/search?q=soldering+iron>`_
* 1x `Soldering wire <https://www.google.com/search?q=soldering+wire>`_
* 1x `3D printer <https://www.google.com/search?q=3D+printer>`_
* 1x `Heat shrink tubes <https://www.google.com/search?q=heat+shrink+tubes>`_
* A **ton of zip ties** and **double sided, high-quality M3 tape** to secure the components in place.
* A **good quality multimeter** to check for shorts and continuity in the circuits.
* M3 lightweight screws and nuts to secure the motors and other components to the frame.

Wiring Diagram
--------------

The wiring diagram for the Pegasus drone is shown below for the Kakute H7-mini v1.3 board. The diagram shows the connections between the Kakute board, the Jetson Orin Nano, the ESC, the motors, the GPS, the battery, the remote control receiver, and altimeter and an optical flow sensor.

.. image:: /_static/vehicles/pegasus/wiring_diagram.png
  :width: 600
  :align: center
  :alt: Wiring diagram

.. admonition:: Kakute H7 v1.3 (regular)

  The general connection diagram for the regular Kakute H7 v1.3 board is similar to the one shown above. Check the official `Holybro website <https://holybro.com/products/kakute-h7>`_ for the pinout of the Kakute H7 (regular size).


CAD Models
----------

* **Version 2 - Carbon Fiber Frame (2024-2025)**

  .. image:: https://github.com/PegasusResearch/pegasus_cad/blob/main/docs/_static/full_assembly_v2.png?raw=true
    :width: 400
    :align: center
    :alt: Pegasus V2 drone prototype

The latest Pegasus Prototype was developed using carbon fiber sheets and 3D printed parts. The carbon fiber sheets were custom cut to the dimensions of the drone frame.

* **Version 1 - 3D Printed Frame (2023-2024)**

  .. image:: https://github.com/PegasusResearch/pegasus_cad/blob/main/docs/_static/full_assembly.png?raw=true
    :width: 400
    :align: center
    :alt: Pegasus V1 drone prototype