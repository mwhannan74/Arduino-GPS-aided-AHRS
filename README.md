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
The IMU is based on the BNO055 AHRS. It uses an Kalamn Filter (EKF) to compute the full 3D orientation of the sensor (you don't have to do this yourself!). You can find my Arduino project for interfacing and calibrating this sensor [here](https://github.com/mwhannan74/Adafruit_BNO055_OrientSensor). That project will allow you to save the calibration and load it at start up. However, you will still need to calibrate the magnetometers on power cycle, but that is super easy (if you can pick your device up) by just moving it back and forth and up and down a little. You should also watch the [Bosch YouTube video](https://www.youtube.com/watch?v=Bw0WuAyGsnY) to understand how the calbration works. My design for this project is to load the values because calibrating the accelerometers is a pain.

## GPS
Key features of the GPS are 10 Hz updates (needed for higher speed motion), GPS + GLONASS support (track up to 33 satellites on 99 channels), high-sensitivity receiver (-165 dBm), RTC battery-compatible (for faster cold starts), Internal patch antenna + connector for externalantenna, and Fix status LED (lets you know what it is doing). 

## Heading Fusion
It may be more precise to call it "IMU heading bias correction." The problem with the IMU's magnetometer-based heading is that it's not accurate to true north due to [magnetic declination](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination). It is however very stable even when stationary (no drift). The GPS returns very accurate true north heading, but only when going faster than about 2 m/s. When stationary, the GPS heading is not defined and will vary dramatically from the actual. I have implemented a simple approach that updates the magnitometer bias (offset from true north) based on GPS heading when going fast enough. The benefit of this approach is the pose heading is updated at the rate of the orienation sensor (up to 100Hz), and only updates the bias (error) estimate of the magnetometer when GPS heading (RMC sentence) is available (1,5,10 Hz).

You can "tune" how much the NEW GPS-based heading is averaged into the current estimated bias using Kn_bias. The smaller Kn_bias the less new measurements are included in the fusion.

Note, that I have not addressed the issue of going backwards with the GPS. **The magnetometer measures the direction the sensor is facing. The GPS measures the direction the sesnor is moving (course over ground - COG).** Right now it attempts to not update the heading bias when it thinks it is going backwards, but I have not dealt with the issue of the IMU and GPS having opposite directions.

## Serial Monitor
I use the serial monitor for all debuging and data logging. Set **bool DEBUG = true;** to get verbose output of what is going on. **bool DEBUG = true;** will only output the final "fused" pose data in a CSV style. You can directly paste the serial monitor's output into a text file and add the .csv file extension. The data format (documented in the code) looks like:
* time_sec, lat_deg, lon_deg, speed_mps, heading_deg, pitch_deg, roll_deg, gps_hdg, imu_hdg, imu_hdg_error, imu_hdg_bias, gpsFixQual, numSat

## Results
My intial testing has proven to be pretty good. The data below is located in he results folder. The images are created from the figures in the MS Excel file.
### Heading
![results](https://github.com/mwhannan74/Arduino-GPS-aided-AHRS/blob/main/results/arduino_GPS_AHRS_data_collect_9-18-22.JPG)
### Bias
![results](https://github.com/mwhannan74/Arduino-GPS-aided-AHRS/blob/main/results/arduino_GPS_AHRS_data_collect_bias_9-18-22.JPG)
