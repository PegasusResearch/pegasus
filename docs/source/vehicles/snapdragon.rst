Snapdragon Drone
================

## Flashing the Pixhawk
TODO

## Setting up the Wifi Connection

### Flash the ESP8266 with the MAVESP8266 firmware

1. Download the MAVESP8266 source code and compile 
```
https://github.com/tridge/mavesp8266
```
or download the latest binary from
```
https://firmware.ardupilot.org/Tools/MAVESP8266/latest/firmware-esp<version>.bin
```

where the version "01_1m" if for boards with 1MB of memory (the typical),  "01" for boards with 512KB of memory and "12e" for boards with 4MB of flash. An "FTDI Cable" is for the first time the firmware is upload to the device. To flash the firmware, follow the steps:

2. solder a 6pin header to the narrow end of the board that has the TX, RX, VCC and GND pins.

3. connect the device to your computer using the FTDI cable

4. put the device into bootloader mode by pressing both the Reset and GPIO0 buttons, then release the Reset button followed by the GPIO0 button. The red LED should remain dimly lit.

5. If you are flashing from a Windows PC the download and run the NodeMCU flasher, available at:

https://github.com/nodemcu/nodemcu-flasher/blob/master/Win64/Release/ESP8266Flasher.exe

and, if flashing from Linux or Mac, use the ESPTool, available at:

https://github.com/espressif/esptool


6. On the Advanced page ensure the Flash size is set to 4MByte, 1MByte or 512KB depending on the board version.

7. On the Config page push the gear and select the firmware downloaded above.

8. On the Operation page select the "COM Port" and push the "Flash" button.  If successful the blue bar will slowly stretch from left to right and the icon on the bottom left will turn green.

<img src="images/esp8266-telemetry-flash.jpg" alt="ESP86266" style="float: left; margin-right: 10px;" />

### Connect the ESP8266 to the Pixhawk
The ESP8266 wifi module is a programmable wifi module that attaches to the Pixhawk. If connected to Serial1/Telem1 these parameters should be set on the autopilot (if using another telemetry port, replace the "1" in the parameter name with the telemetry port's number):

- `SERIAL1_PROTOCOL <SERIAL1_PROTOCOL>` = 2 (MAVLink2) or 1 (MAVLink1)
- `SERIAL1_BAUD <SERIAL1_BAUD>` = 921 (921600 baud)

If you have problems connecting, it may help to set:
- `BRD_SER1_RTSCTS <BRD_SER1_RTSCTS>` = 0

to disable flow control although this is not normally necessary.

<img src="images/esp8266-telemetry-pixhawk.jpg" alt="ESP86266 and Pixhawk" style="float: left; margin-right: 10px;" />

 Connect from QGroundControl

1. On your PC settings, open the wifi network connections screen and select the wifi access point (SSID "ArduPilot" with a lower case password "ardupilot").

2. On QGroundControl, set the connection type to UDP and press "Connect"

### Changing the Wifi SSID and Password

1. Connect from your PC to the wifi access point (initial access point ID is "ArduPilot", and password is "ardupilot")
2. Open a browser to `192.168.4.1`
3. Click on the "Setup" link
4. Set the "AP SSID" and "AP Password" fields, push "Save" and reboot.

<img src="images/esp8266-telemetry-web-setup.png" alt="ESP86266" style="float: left; margin-right: 10px;" />

This setup page was adpated from the following sources: 
- https://ardupilot.org/copter/docs/common-esp8266-telemetry.html
- https://rays-blog.de/2016/10/21/224/adding-wi-fi-telemetry-to-pixhawk-flight-controller-with-esp8266-module

### Selecting the Wifi Mode

The wifi card (ESP8266) can work in "client mode" or "access point".

- Client mode - The ESP8266 connects to a pre-specified wifi network and sends/receives data in the UDP port specified in the QGroundControl configuration page. This modes is usefull if we have a "router/access-point" nearby that we can connect to, allowing for a developer to test code while maintaing and active connection of the PC to the network.

- Access point - The ESP8266 acts as a 'router/access-point' to which your PC connects to (the default network SSID is 'ardupilot'). Afterwards, your PC will be able to send/receive data in the UDP port specified in QGroundControl, but will not have access to the internet. This mode is usefull, for example if flying outside where a router is not available.

This setup can be performed by accessing the IP `192.168.4.1` or QGRoundControl configurations.

### Setting up the MAVLink UDP Port
TODO

sudo tcpdump -n -i any