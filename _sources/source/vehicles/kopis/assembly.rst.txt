Drone Assembly
==============

Bill of Materials
-----------------

In order to replicate the Kopis setup adopted on the Taguspark Flight Arena, the following components are required:

* 1x `Kopis Cinewhoop 3 (without camera & VTX) <https://holybro.com/products/kopis-cinewhoop-3-analog-vtx-version>`__
* 1x `ESP 07 with external antenna <https://pt.aliexpress.com/item/32995506222.html?spm=a2g0o.productlist.main.33.6327tsF8tsF80H&algo_pvid=543de1e9-f2e1-4fa6-a3b8-6930dfbaca34&algo_exp_id=543de1e9-f2e1-4fa6-a3b8-6930dfbaca34-16&pdp_npi=4%40dis%21EUR%211.22%211.22%21%21%211.27%211.27%21%40210312d517134561652431119e9e17%2112000031251421154%21sea%21PT%210%21AB&curPageLogUid=k6p8cp4UFkJJ&utparam-url=scene%3Asearch%7Cquery_from%3A>`__
* 1x `Lipo Tattu Battery 4S 1300mAh <https://rc-innovations.es/shop/bateria-lipo-tattu-4s-1300mah-100c-TA-FF-100C-1300-4S1P?category=16>`__
* 1x `FrSky XM Plus ACCST 16CH Sbus <https://www.frsky-rc.com/product/xm-plus/>`_
* 1x `Time-of-flight sensor <https://holybro.com/collections/sensors/products/st-vl53l1x-lidar>`__
* 1x `Optical flow sensor <https://holybro.com/products/pmw3901-optical-flow-sensor>`__

Wiring Diagram
------------------
TODO


Kakute Flight Controller Setup
------------------------------

Two different approaches are considered to setup the Kakute flight controller: 1) using our custom precompiled binaries (recommended), or 2) compiling PX4 from source.

**Option 1 - Using custom precompiled binaries (recommended)**

We provide some precompiled binaries of the PX4-autopilot source code (release v1.14.2), with some minor corrections and modifications for our specific use case.
To flash these custom images, proceed as follows.

1. Install DFU-Util to flash the bootloader into the Kakute board:

  .. code:: bash

      sudo apt-get install dfu-util

2. Open a terminal, and download the bootloader images into your computer, depending on the required board version. Do not close the terminal once you finish.

  .. code:: bash
    
      # Use these command for the KakuteH7
      wget "https://github.com/PegasusResearch/drone_configs/raw/refs/heads/main/Kopis/PX4-Autopilot(v1.14.2)/KakuteH7/holybro_kakuteh7_bootloader.bin"

      # Use this command for the KakuteH7v2
      wget "https://github.com/PegasusResearch/drone_configs/raw/refs/heads/main/Kopis/PX4-Autopilot(v1.14.2)/KakuteH7v2/holybro_kakuteh7v2_bootloader.bin"

3. Connect the Kakute to the computer using a USB cable and put the board in bootloader mode by pressing the button on the board while connecting the cable. Flash the bootloader into the Kakute:

  .. code:: bash

      # Use these commands for the KakuteH7
      dfu-util -a 0 --dfuse-address 0x08000000:force:mass-erase:leave -D ./holybro_kakuteh7_bootloader.bin
      dfu-util -a 0 --dfuse-address 0x08000000 -D  ./holybro_kakuteh7_bootloader.bin

      # Use these commands for the KakuteH7v2
      dfu-util -a 0 --dfuse-address 0x08000000:force:mass-erase:leave -D ./holybro_kakuteh7v2_bootloader.bin
      dfu-util -a 0 --dfuse-address 0x08000000 -D  ./holybro_kakuteh7v2_bootloader.bin

5. After flashing the bootloader, reboot the Kakute without pressing the button. Download the firmware for the `KakuteH7 <https://github.com/PegasusResearch/drone_configs/raw/refs/heads/main/Kopis/PX4-Autopilot(v1.14.2)/KakuteH7/holybro_kakuteh7_default.px4>`__ or for the `KakuteH7v2 <https://github.com/PegasusResearch/drone_configs/raw/refs/heads/main/Kopis/PX4-Autopilot(v1.14.2)/KakuteH7v2/holybro_kakuteh7v2_default.px4>`__ and load it to the board using QGroundControl (instrutions `here <https://docs.px4.io/main/en/config/firmware.html#loading-firmware>`__). Note that you want to install a custom version (specific instructions `here <https://docs.px4.io/main/en/config/firmware.html#installing-px4-main-beta-or-custom-firmware>`__).

6. After having the firmware installed, connect the Kakute to the computer and open QGroundControl.

7. Load the Kopis parameters from the configuration file `kopis7.params <https://github.com/PegasusResearch/drone_configs/blob/main/Kopis/Parameters/kopis_7.params>`__.

8. Change the MAV_SYS_ID parameter to the ID of the new vehicle.



**Option 2 - Compiling PX4 from source**

1. Configure the KakuteH7/KakuteH7v2 flight controller with PX4 firmware (v1.14.2), by following the instructions on the `PX4 documentation <https://docs.px4.io/main/en/flight_controller/kakuteh7v2.html>`__. Start by cloning and compiling the PX4 firmware repository:

  .. code:: bash

      # Clone PX4 firmware repository
      git clone https://github.com/PX4/PX4-Autopilot.git
      cd PX4-Autopilot
      git checkout v1.14.2

2. Compile the bootloader and the firmware for the flight controller. Use the commands depending on the board version (KakuteH7/KakuteH7v2):

  .. code:: bash

      # Use these commands to compile the bootloader and the firmware for the KakuteH7
      make holybro_kakuteh7_bootloader
      make holybro_kakuteh7_default

      # Use these commands to compile the bootloader and the firmware for the KakuteH7v2
      make holybro_kakuteh7v2_bootloader
      make holybro_kakuteh7v2_default

3. Install DFU-Util to flash the bootloader into the Kakute board:

  .. code:: bash

      sudo apt-get install dfu-util

4. Connect the Kakute to the computer using a USB cable and put the board in bootloader mode by pressing the button on the board while connecting the cable. Flash the bootloader into the Kakute:

  .. code:: bash

      # Use these commands for the KakuteH7
      dfu-util -a 0 --dfuse-address 0x08000000:force:mass-erase:leave -D build/holybro_kakuteh7_bootloader/holybro_kakuteh7_bootloader.bin
      dfu-util -a 0 --dfuse-address 0x08000000 -D  build/holybro_kakuteh7_bootloader/holybro_kakuteh7_bootloader.bin

      # Use these commands for the KakuteH7v2
      dfu-util -a 0 --dfuse-address 0x08000000:force:mass-erase:leave -D build/holybro_kakuteh7v2_bootloader/holybro_kakuteh7v2_bootloader.bin
      dfu-util -a 0 --dfuse-address 0x08000000 -D  build/holybro_kakuteh7v2_bootloader/holybro_kakuteh7v2_bootloader.bin

5. After flashing the bootloader, reboot the Kakute without pressing the button. Upload the firmware by using QGroundControl or run one of the following commands

  .. code:: bash
    
    # Use this command for the KakuteH7
    make holybro_kakuteh7_default upload

    # Use this command for the KakuteH7v2
    make holybro_kakuteh7v2_default upload

6. After having the firmware installed, connect the Kakute to the computer and open QGroundControl.

7. Load the Kopis parameters from the configuration file `kopis7.params <https://github.com/PegasusResearch/drone_configs/blob/main/Kopis/kopis_7.params>`__.

8. Change the MAV_SYS_ID parameter to the ID of the new vehicle.

ESP Configuration
-----------------

1. Connect the USB-FDTI cable to the ESP, and turn the switch into the ``PROGRAM`` position.

  .. admonition:: Warning

    Check whether your ESP runs with 3.3V or 5V before connecting the FDTI cable

  .. image:: /_static/vehicles/kopis/esp8266-connection_pinout.jpeg

2. Install the esptool by running the following command:

  .. code:: bash

    pip install esptool

3. Download the custom compiled firmware from `here <https://github.com/PegasusResearch/drone_configs/tree/main/ESP8266>`__. 

4. Erase the ESP flash memory by running the following command:

  .. code:: bash

    esptool.py --baud 921600 --port /dev/ttyUSB0 erase_flash  

  .. image:: /_static/vehicles/kopis/esptool_erase_flash.png
    :width: 600px
    :align: center
    :alt: Erasing the ESP flash memory

5. Flash the firmware into the ESP using the following command:

  .. code:: bash
    
    esptool.py --baud 921600 --port /dev/ttyUSB0 write_flash 0x00000 firmware-1.2.2.bin

  .. image:: /_static/vehicles/kopis/esptool_write_flash.png
    :width: 600px
    :align: center
    :alt: Write the ESP flash memory

6. Reboot the ESP with the switch in the ``UART`` position.
7. Connect the computer to the ``PixRacer`` wifi network generated by the ESP. The password is ``pixracer``.

  .. image:: /_static/vehicles/kopis/pixracer_select_wifi_network.png
    :width: 200px
    :align: center
    :alt: ESP wifi network

8. On your browser, go to the IP address ``192.168.4.1`` and click on the ``Setup`` link. This will open a page with the default configurations.

  .. image:: /_static/vehicles/kopis/mavlink_wifi_bridge_default.png
    :width: 300px
    :align: center
    :alt: ESP setup page

9. Change the configurations according to the image bellow, by setting the ``UDP Port``, ``AP SSID``and ``Station IP`` according to the standard adopted in the table above.

  .. image:: /_static/vehicles/kopis/mavlink_wifi_bridge_station_params.png
    :width: 300px
    :align: center
    :alt: ESP configuration

If you have any questions or need help with the setup, please check the `ardupilot reference page <https://ardupilot.org/copter/docs/common-esp8266-telemetry.html>`__.