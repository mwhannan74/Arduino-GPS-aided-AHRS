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

## IMU
The IMU is based on the BNO055 AHRS. It is a fully EKF based sensor that computes the full 3D orientation of the sensor. You can find my Arduino project for interfacing and calibrating this sensor [here](https://github.com/mwhannan74/Adafruit_BNO055_OrientSensor). This will allow you to save the calibration and load it at start up. Hoever, you will still need to calibrate the magnetometers on power cycle, but that is super easy (if you can pick your device up) by just moving it back and forth and up and down a little. You should also watch the [Bosch YouTube video](https://www.youtube.com/watch?v=Bw0WuAyGsnY) to understand how the calbration works. My design for this project is to load the values.

## Heading Fusion
It may be more precise to call it "IMU heading bias correction." The problem with the magnetometer heading it that it is not accurate to true north due to [magnetic declination](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination), but it is very stable even when stationary. The GPS returns very accurate true north heading BUT only when going faster than 2 m/s. When stationary, the GPS heading is not defined and will vary dramatically from the actual. I have implemented a simple approach that updates the magnitometer bias (offset from true north) based on GPS heading when going fast enough. The benefit of this approach is the pose heading is updated at the rate of the orienation sensor (up to 100Hz), and only updates the bias (error) estimate of the magnetometer when GPS heading (RMC sentence) is available (1,5,10 Hz).
