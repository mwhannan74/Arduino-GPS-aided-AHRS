// GPS
#include <Adafruit_GPS.h>
#include <vector>
#include <string>

// BNO055 Orient Sensor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


//============================================================================================
// HELPER FUNCTIONS
//============================================================================================
static const float D2R = PI/180.0;
static const float R2D = 180.0/PI;
static const float KNOTS2MPS = 0.514;

float angleWrap_deg(float angle)
{
  float aw = angle;
  if( angle > 180.0 ) aw = angle - 360.0;
  else if( angle < -180.0 ) aw = angle + 360;
  return aw;
}

unsigned long timeStart_ms = 0;


//============================================================================================
// Onboard LED
//============================================================================================
static const int ledPin = 13;
bool ledOn = true;


//============================================================================================
// Adafruit Ultimate GPS 
//============================================================================================
#define SerialGPS Serial1
static const uint32_t GPS_BAUD = 19200; // default is 9600
static const int GPS_RATE = 10; // 10, 5, 1
bool isGGA = false;

Adafruit_GPS GPS(&SerialGPS);
static const std::vector<std::string> GpsFixQuality{"invalid", "SPS", "DGPS", "PPS", "RTK", "fRTK", "dead reck", "manual", "sim"};

void setupGPS()
{
  // GPS Serial setup
  SerialGPS.begin(GPS_BAUD); // Updated baud

  // Always request RMC (does not include altitude or satellite info)
  // If baud rate is high enough or rate is slow enough request both GGA and RMC
//  if( GPS_RATE == 10 )
//  {
//    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
//    isGGA = false;
//  }
//  else
//  {
//    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//    isGGA = true;
//  }
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  isGGA = true;
 
  // Set the update rate
  if( GPS_RATE == 10 )      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate - can only output one message (RMC) even if you up the baud rate  
  else if( GPS_RATE == 5 )  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);  // 5 Hz update rate
  else                      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate  
  
  delay(500);
}


// Parse -- read data from the GPS in the 'main loop'
// returns an int value that corresponds to: NA=0, FAIL=1, RMC=2, GGA=3. OTHER=4 (can't use an emum because the Arduino IDE reorders enums after functions)
int readGPS()
{  
  char c = GPS.read();
  
  if( GPS.newNMEAreceived() ) 
  {
    int rVal = 1;
    if( strcmp(GPS.thisSentence,"RMC") == 0 ) rVal = 2;
    else if( strcmp(GPS.thisSentence,"GGA") == 0 ) rVal = 3;
    else rVal = 4;

    // Now actually parse the message
    if( !GPS.parse(GPS.lastNMEA()) ) // parse() sets the newNMEAreceived() flag to false
    {
      // we can fail to parse a sentence in which case we should just wait for another
      return 1; // FAIL
    }
    return rVal;    
  }
  return 0; // NA
}

void printGPS()
{
  Serial.print("Fix: ");
  if( (int)GPS.fix ) Serial.println("True");
  else Serial.println("False");
  
  Serial.print("Fix quality: "); Serial.println( GpsFixQuality[(int)GPS.fixquality].c_str() );
  
  if( GPS.fix ) 
  {
    // decimal
    // places   degrees          distance
    // -------  -------          --------
    // 0        1                111  km
    // 1        0.1              11.1 km
    // 2        0.01             1.11 km
    // 3        0.001            111  m
    // 4        0.0001           11.1 m
    // 5        0.00001          1.11 m
    // 6        0.000001         11.1 cm
    // 7        0.0000001        1.11 cm
    // 8        0.00000001       1.11 mm
    Serial.print("Position: ");
    Serial.print(GPS.latitudeDegrees,11); Serial.print(", ");     
    Serial.println(GPS.longitudeDegrees,11);
    
    Serial.print("Altitude MSL (m): "); Serial.println(GPS.altitude);
    Serial.print("Geoid Height (m): "); Serial.println(GPS.geoidheight);

    Serial.print("Speed (m/s): "); Serial.println(GPS.speed*KNOTS2MPS);
    Serial.print("Course (deg): "); Serial.println(GPS.angle);
    //Serial.print("Magnetic Var (deg): "); Serial.println(GPS.magvariation); // should always be zero --> GPS always points true North
    
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);      
    Serial.println("");
  }
}



//============================================================================================
// Adafruit BNO055 IMU Orient Sensor
//============================================================================================
Adafruit_BNO055 bno = Adafruit_BNO055(55);
struct
{
  bool isCalibrated = false;
  float hdg_mag = 0;
  float hdg_true = 0;
  float pitch = 0;
  float roll = 0;
  float declination = -10.8186;
} IMU;

// Display sensor calibration status
// The four calibration registers (an overall system calibration status, 
// as well individual gyroscope, magnetometer and accelerometer values)
// will return a value between '0' (uncalibrated data) and '3' (fully calibrated). 
// The higher the number the better the data will be.
bool displayImuCalibStatus(void)
{  
  uint8_t sys, gyro, accel, mag;
  sys = gyro = accel = mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("Sys = "); Serial.print(sys, DEC);
  Serial.print(" G = "); Serial.print(gyro, DEC);
  Serial.print(" A = "); Serial.print(accel, DEC);
  Serial.print(" M = "); Serial.println(mag, DEC);

  if( ((gyro+accel+mag) == 9) && (sys == 3)) return true;
  else return false;
}

// Display the raw calibration offset and radius data
// Formated so you can just past directly into loadImuConfig()
void displayImuCalibOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.println("\nBNO055 Calibration Data");
    Serial.print("calibData.accel_offset_x = "); Serial.print(calibData.accel_offset_x); Serial.println(";");
    Serial.print("calibData.accel_offset_y = "); Serial.print(calibData.accel_offset_y); Serial.println(";");
    Serial.print("calibData.accel_offset_z = "); Serial.print(calibData.accel_offset_z); Serial.println(";");
    Serial.print("calibData.accel_radius = "); Serial.print(calibData.accel_radius); Serial.println(";");

    Serial.print("calibData.gyro_offset_x = "); Serial.print(calibData.gyro_offset_x); Serial.println(";");
    Serial.print("calibData.gyro_offset_y = "); Serial.print(calibData.gyro_offset_y); Serial.println(";");
    Serial.print("calibData.gyro_offset_z = "); Serial.print(calibData.gyro_offset_z); Serial.println(";");

    Serial.print("calibData.mag_offset_x = "); Serial.print(calibData.mag_offset_x); Serial.println(";");
    Serial.print("calibData.mag_offset_y = "); Serial.print(calibData.mag_offset_y); Serial.println(";");
    Serial.print("calibData.mag_offset_z = "); Serial.print(calibData.mag_offset_z); Serial.println(";");
    Serial.print("calibData.mag_radius = "); Serial.print(calibData.mag_radius); Serial.println(";");

    Serial.println("");
}

// This will eliminate the need to calibrate the accelerometers!!!
// The magnetometers will still need to be calibrated
// The gyros still require the sensor to be motionless at start up
bool loadImuCalibration()
{
  adafruit_bno055_offsets_t calibData;
  bno.getSensorOffsets( calibData );
  displayImuCalibOffsets( calibData );
  
  Serial.println("Restoring calibration data to the BNO055");  
  calibData.accel_offset_x = 16;
  calibData.accel_offset_y = -23;
  calibData.accel_offset_z = -5;
  calibData.accel_radius = 1000;
  
  calibData.gyro_offset_x = 1;
  calibData.gyro_offset_y = -2;
  calibData.gyro_offset_z = 1;
  
  calibData.mag_offset_x = -122;
  calibData.mag_offset_y = 424;
  calibData.mag_offset_z = -163; 
  calibData.mag_radius = 778;  
  
  bno.setSensorOffsets(calibData);
  displayImuCalibOffsets( calibData );

  return true;
}

void setupIMU()
{
  // Initialize the BNO055 sensor 
  if(!bno.begin())  
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("ERROR, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1)
    {
      delay(10);
    }
  }  
  delay(500);    
  
  bno.setExtCrystalUse(true); 
  delay(500);

  //*****************************************************************************************
  // For a fast calibration, uncomment this line and use the most recently saved calibration.
  // You will still need to calibrate the magnetometers by mvoing it around.
  loadImuCalibration();
  //*****************************************************************************************

  while( !IMU.isCalibrated )
  {
    IMU.isCalibrated = displayImuCalibStatus();      
  }
  adafruit_bno055_offsets_t calibData;
  bno.getSensorOffsets( calibData );
  displayImuCalibOffsets( calibData );
}


// Data is always available on a call to read since (I believe) the library
// driectly requests the data from the sensor. The sensor supports Euler vector
// at 100Hz. This is unlike the GPS NMEA strings that need to be parsed one 
// character at a time.
void readIMU()
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2 (100Hz)
  // - VECTOR_MAGNETOMETER  - uT (20Hz)
  // - VECTOR_GYROSCOPE     - rad/s (100Hz)
  // - VECTOR_EULER         - degrees (100Hz)
  // - VECTOR_LINEARACCEL   - m/s^2 (100Hz)
  // - VECTOR_GRAVITY       - m/s^2 (100Hz)
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // P1 (default)
  IMU.hdg_mag = euler.x() + 90.0; // CW is positive; seems to be off 90 deg
  IMU.pitch = euler.y();  // up is positive
  IMU.roll = euler.z();   // CCW (left) is positive

  // angle wrap
  IMU.hdg_mag = angleWrap_deg(IMU.hdg_mag);
  IMU.pitch = angleWrap_deg(IMU.pitch);
  IMU.roll = angleWrap_deg(IMU.roll);

  // declination adjustment
  IMU.hdg_true = IMU.hdg_mag - IMU.declination;
  IMU.hdg_true = angleWrap_deg(IMU.hdg_true);
}

void printIMU()
{
  Serial.print("hdg_mag:");
  Serial.print(IMU.hdg_mag, 4);
  Serial.print(" hdg_true:");
  Serial.print(IMU.hdg_true, 4);
  Serial.print(" pitch:");
  Serial.print(IMU.pitch, 4);
  Serial.print(" roll:");
  Serial.print(IMU.roll, 4);
  Serial.println("");
}


//============================================================================================
// SETUP
//============================================================================================
void setup() 
{ 
  pinMode(ledPin, ledOn);
  Serial.begin(115200);
  setupGPS();
  setupIMU();

  timeStart_ms = millis();
}


//============================================================================================
//============================================================================================
// LOOP
//============================================================================================
//============================================================================================
unsigned long blinkPeriod_ms = 1000; // 1Hz
unsigned long fusionPeriod_ms = 50;  // 20Hz (IMU can go up to 100Hz)
unsigned long printPeriod_ms = 200;  // 5Hz

uint32_t time_ms = millis();
uint32_t timeLED_ms = millis();
uint32_t timeFusion_ms = millis();
uint32_t timePrint_ms = millis();

// The smaller Kn_bias the less new measurements are included in the fusion
float Kn_bias = 0.2; // must be 0.0 < Kn_bias < 1.0 
float Ko_bias = 1.0 - Kn_bias;

struct
{
  float time_sec = 0;
  float lat_deg = 0;
  float lon_deg = 0;
  float speed_mps = 0;
  float heading_deg = 0;
  float pitch_deg = 0;
  float roll_deg = 0;  

  // need to record for validating heading fusion
  float gps_hdg = 0;
  float imu_hdg = 0;
  float imu_hdg_error = 0;
  float imu_hdg_bias = 0;

  // Useful GPS info (when available)
  std::string gpsFixQual = "NA";
  unsigned int numSat = 0;  


  // print in CSV format
  // time_sec, lat_deg, lon_deg, speed_mps, heading_deg, pitch_deg, roll_deg, gps_hdg, imu_hdg, imu_hdg_error, imu_hdg_bias, gpsFixQual, numSat
  void print()
  {
    // time_sec, lat_deg, lon_deg, speed_mps, heading_deg, pitch_deg, roll_deg
    Serial.print(time_sec); Serial.print(", ");
    Serial.print(lat_deg,11); Serial.print(", ");
    Serial.print(lon_deg,11); Serial.print(", ");
    Serial.print(speed_mps); Serial.print(", ");
    Serial.print(heading_deg); Serial.print(", ");
    Serial.print(pitch_deg); Serial.print(", ");    
    Serial.print(roll_deg); Serial.print(", ");    

    // gps_hdg, imu_hdg, imu_hdg_error, imu_hdg_bias
    Serial.print(gps_hdg); Serial.print(", ");
    Serial.print(imu_hdg); Serial.print(", ");
    Serial.print(imu_hdg_error); Serial.print(", ");
    Serial.print(imu_hdg_bias);

    // optional: gpsFixQual, numSat
    if( isGGA )
    {
      Serial.print(", ");
      Serial.print(gpsFixQual.c_str()); Serial.print(", ");
      Serial.print(numSat);
    }
    
    Serial.println("");
  }
} pose;

bool DEBUG = false;

void loop() 
{  
  // each loops takes around 1-2ms to execute --> means that we are processing serial data at around 500Hz
  unsigned long etime_ms = millis() - time_ms;
  float etime_sec = 0.001*(float)etime_ms;
  float loopRate_Hz = 1.0 / etime_sec;

  // get the current time in msec for this loop
  // Needs to go after the etime calc
  time_ms = millis();
  

  //------------------------------------------------------------
  // Get GPS data from serial port on every loop because it needs to be parsed
  int gpsParseStatus = readGPS();
//  if( gpsParseStatus != 0 )
//  {
//    Serial.print("gpsParseStatus = "); Serial.println(gpsParseStatus);
//  }


  //------------------------------------------------------------
  // Create the fused pose data from the IMU and GPS
  // This needs to run at a slower rate that the GPS sentence processing loop
  if( time_ms - timeFusion_ms > fusionPeriod_ms )
  {
    timeFusion_ms = time_ms;

    // Get IMU data from I2C port
    readIMU();
  
    // all values except heading do not need to be fused because we do not have a motion model yet
    pose.time_sec = 0.001 * float(time_ms - timeStart_ms);
    pose.lat_deg = GPS.latitudeDegrees;
    pose.lon_deg = GPS.longitudeDegrees;
    pose.speed_mps = GPS.speed*KNOTS2MPS;
    pose.pitch_deg = IMU.pitch;
    pose.roll_deg = IMU.roll;
    if( isGGA )
    {
      pose.gpsFixQual = GpsFixQuality[(int)GPS.fixquality];
      pose.numSat = GPS.satellites;      
    }

    // Heading Fusion -> It may be more precise to call it "IMU heading bias correction."
    // This is a simple approach that just updates the magnitometer bias based on GPS heading.
    // The benefit of this approach is the pose heading is updated at the rate of the orienation sensor (up to 100Hz),
    // and only updates the bias (error) estimate when GPS heading (RMC sentence) is available (1,5,10 Hz).
    pose.gps_hdg = angleWrap_deg(GPS.angle);
    pose.imu_hdg = angleWrap_deg(IMU.hdg_true);
    pose.heading_deg = angleWrap_deg(pose.imu_hdg + pose.imu_hdg_bias);
  }


  //------------------------------------------------------------
  // Event driven update of the IMU's heading bias the instant that we get a new RMC message from the GPS
  // NA=0, FAIL=1, RMC=2, GGA=3
  if( gpsParseStatus == 2 )
  {
    //Serial.println("IMU Bias --> RMC message parsed");

    // GPS Heading is not valid under 1.15 m/s
    if( pose.speed_mps > 2.0)
    {
      //Serial.println("IMU Bias --> GPS speed > 2");
      
      // simple averaging filter for the bias
      // only update bias if error is relatively small, this helps deal with reverse motion of the GPS corrupting the bias estimate
      float error = pose.gps_hdg - pose.imu_hdg;
      if( fabs(error) < 45.0 )
      { 
        pose.imu_hdg_error = error;
        pose.imu_hdg_bias = Ko_bias*pose.imu_hdg_bias + Kn_bias*pose.imu_hdg_error;
      }
    }     
  }
  

  //------------------------------------------------------------
  // Print data
  if( time_ms - timePrint_ms > printPeriod_ms ) 
  {
    timePrint_ms = time_ms;
   
    if( DEBUG )
    {
      Serial.println("\n-------------------------------------------------");
      Serial.print("etime_ms: "); Serial.print(etime_ms);
      Serial.print(" etime_sec: "); Serial.print(etime_sec,5);
      Serial.print(" loopRate_Hz: "); Serial.println(loopRate_Hz);
      Serial.println("");  
  
      printGPS();
      
      printIMU();  
    }

    pose.print();    
  }


  //------------------------------------------------------------
  // Blink LED
  if( time_ms - timeLED_ms > blinkPeriod_ms ) 
  {
    timeLED_ms = time_ms;
    ledOn = !ledOn;
    digitalWrite(ledPin, ledOn);
  }

  // no delay because we need to run readGPS as fast as possible to parse NMEA sentances!
}
