Kopis 3"
========

.. image:: https://holybro.com/cdn/shop/products/30064_2_1800x1800.jpg?v=1647227793
  :width: 600
  :align: center
  :alt: Kopis 3 drone

Bill of Materials
-----------------

In order to replicate the Pegasus Drone, the following components are required:

* 1x `Kopis Cinewhoop 3 (without camera & VTX) <https://holybro.com/products/kopis-cinewhoop-3-analog-vtx-version>`__
* 1x `ESP 07 with external antenna <https://pt.aliexpress.com/item/32995506222.html?spm=a2g0o.productlist.main.33.6327tsF8tsF80H&algo_pvid=543de1e9-f2e1-4fa6-a3b8-6930dfbaca34&algo_exp_id=543de1e9-f2e1-4fa6-a3b8-6930dfbaca34-16&pdp_npi=4%40dis%21EUR%211.22%211.22%21%21%211.27%211.27%21%40210312d517134561652431119e9e17%2112000031251421154%21sea%21PT%210%21AB&curPageLogUid=k6p8cp4UFkJJ&utparam-url=scene%3Asearch%7Cquery_from%3A>`__
* 1x `Lipo Tattu Battery 4S 1300mAh <https://rc-innovations.es/shop/bateria-lipo-tattu-4s-1300mah-100c-TA-FF-100C-1300-4S1P?category=16>`__
* 1x `FrSky XM Plus ACCST 16CH Sbus <https://www.frsky-rc.com/product/xm-plus/>`_
* 1x `Time-of-flight sensor <https://holybro.com/collections/sensors/products/st-vl53l1x-lidar>`__
* 1x `Optical flow sensor <https://holybro.com/products/pmw3901-optical-flow-sensor>`__

Kopis Drone Setup
-----------------

ESP Configuration
-----------------

1. Connect the FDTI cable to the ESP

.. image:: images/esp8266-connection_pinout.jpeg

2. Check whether your ESP runs with 3.3V or 5V

3. Download the custom firmware from (TODO)

.. code::
	
	esptool.py --baud 921600 --port /dev/ttyUSB0 write_flash 0x00000 firmware-1.2.2.bin

