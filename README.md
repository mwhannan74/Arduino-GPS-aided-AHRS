# Arduino-GPS-aided-AHRS
An Arduino based GPS-Aided Attitude Heading Reference System (AHRS).

This is a personal project that I put together that combines a GPS and 9-DOF IMU using a Arduino Teensy 3.1. The main contributions of this work are:
1. Interface to GPS.
2. Interface to IMU.
3. "Fusion" of IMU heading with GPS heading  (more details below)

## The hardware components of the design are:
* [Adafruit Ultimate GPS Breakout with GLONASS + GPS - PA1616D - 99 channel w/10 Hz updates](https://www.adafruit.com/product/5440)
  * [GPS Library install](https://learn.adafruit.com/adafruit-ultimate-gps/arduino-wiring)
* [Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055](https://www.adafruit.com/product/2472)
  * [IMU Library Install](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code)
* [Teensy 3.1 Development Board](https://www.pjrc.com/store/teensy32.html)
