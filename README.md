# ESP32-Quadcopter


### Introduction

**ESP32-Quadcopter** is an open source solution based on Espressif ESP32/ESP32-S2/ESP32-S3 Wi-Fi chip, which can be controlled by a mobile APP or gamepad over **Wi-Fi** connection. ESP-Drone comes with **simple hardware**, **clear and extensible code architecture**, and therefore this project can be used in **STEM education** and other fields. The main code is ported from **Crazyflie** open source project with **GPL3.0** protocol.

> Currently support ESP32, ESP32S2, ESP32S3, please using ESP-IDF [release/v4.4](https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/get-started/index.html) branch as your develop environment

![ESP-Drone](./pics/drone1.jpg)

For more information, please check the sections below:
* **Getting Started**: 
* **Hardware Schematic**：
* **iOS APP Source code**: [ESP-Drone-iOS](https://github.com/EspressifApps/ESP-Drone-iOS)
* **Android APP Source code**: [ESP-Drone-Android](https://github.com/EspressifApps/ESP-Drone-Android)

### Features

1. Stabilize Mode
2. Height-hold Mode
3. Altitude-hold Mode
4. Position-hold Mode
5. APP Control
6. CFclient Supported
7. ESP-BOX3 Joystick Control (through esp-now)

Note: to implement Altitude-hold/Height-hold/Position-hold mode, extension boards are needed. For more information, see Hardware Reference. 

### Third Party Copyrighted Code

Additional third party copyrighted code is included under the following licenses.

| Component | License | Origin |Commit ID |
| :---:  | :---: | :---: |:---: |
| core/crazyflie | GPL3.0  |[Crazyflie](https://github.com/bitcraze/crazyflie-firmware) |tag_2021_01 b448553|
| lib/dsp_lib |  | [esp32-lin](https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib) |6fa39f4c|

### Support Policy

Limited Support

### THANKS

1. Thanks to Bitcraze for the great [Crazyflie project](https://www.bitcraze.io/%20).
2. Thanks to Espressif for the powerful [ESP-IDF framework](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html).
3. Thanks to WhyEngineer for the useful [ESP-DSP lib](https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib).

