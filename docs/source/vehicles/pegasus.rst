Pegasus Drone v1.0.0
====================

CAD Model
---------
The lastest CAD model for the Pegasus Drone can be found in the `Pegasus CAD <https://github.com/PegasusResearch/pegasus_cad>`_ repository under a `Creative Commons Non-Commercial & Non-Military License <https://github.com/PegasusResearch/pegasus_cad/blob/main/LICENSE>`_. 

.. image:: https://github.com/PegasusResearch/pegasus_cad/blob/main/docs/_static/full_assembly.png?raw=true
  :width: 600
  :align: center
  :alt: Pegasus Drone CAD Model

Bill of Materials
-----------------

In order to replicate the Pegasus Drone, the following components are required:

* 1x `Nvidia Jetson Orin Nano Developer Kit <https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit>`_
* 1x 256GB NVMe SSD PCIe 4.0
* 1x `Kakute H7 v1.3 stack (Microcontroller + 4in1 ESC) <https://holybro.com/products/kakute-h7-v1-stacks?variant=42833125277885>`_ 
* 4x `T-motor V2306 V2 2400KV <https://store.tmotor.com/product/v2306-v2-fpv-motor.html>`_
* 1x `Micro M8N GPS <https://holybro.com/collections/standard-gps-module/products/micro-m8n-gps>`_
* 1x `FrSky XM Plus ACCST 16CH Sbus <https://www.frsky-rc.com/product/xm-plus/>`_
* 1x `Realsense d435i <https://www.intelrealsense.com/depth-camera-d435i/>`_
* 4x `Propellers Dalprop Cyclone T5050 <http://www.dalprop.com>`_
* 1x Battery 4S 4300mAh
* 1x Set of jumper dupont wires
* 1x XT60 cable connector

PX4 Configuration for Indoor Flight
-----------------------------------

Jetson Orin Nano Setup
----------------------

In order to setup the Jetson Orin Nano Developer Kit, follow the instructions bellow. 

1. Start by installing the `Nvidia SDK manager <https://developer.nvidia.com/sdk-manager>`__ on a machine with Ubuntu 22.04LTS or later.
2. Connect the Jetson Orin Nano Developer Kit to the machine using a USB-C cable.

You can find more information on the `Nvidia Jetson Orin Nano Developer Kit <https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit>`_ page.

Pegasus Drone Setup
-------------------

Realsense Setup
---------------

