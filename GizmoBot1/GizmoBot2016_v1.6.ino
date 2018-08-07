/**
   Arduino code for an autonomous vehicle.

   Author: Andy Grove

   License: None. Feel free to use in your projects.
*/

/* mark notes
  Ping on interrupts
  Weighted direction choices for left/right/halt determination



*/

#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include <SPI.h>
//3rd party library: Adafruit Unified Sensor library from: https://github.com/adafruit/Adafruit_Sensor
//#include <Adafruit_Sensor.h>
//3rd party library: Adafruit GPS library from: https://github.com/adafruit/Adafruit_GPS
//#include <Adafruit_GPS.h>
//#include <SoftwareSerial.h>

//3rd party library: NewPing from: https://bitbucket.org/teckel12/arduino-new-ping/downloads
#include <NewPing.h>
//3rd party library: LSM303 from: https://github.com/pololu/lsm303-arduino
//#include <LSM303.h>
//3rd party library: TinyGPS++ from: http://arduiniana.org/libraries/tinygpsplus/
#include <TinyGPS++.h>
//3rd party library: SparkFun LSM9DS1 from: ???
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>


//#include "TGD_Movement.h" // Set pin assignments in "TGD_Movement.h"
#include "location.h"

//Pinout:
#define goPin 53 //Flip switch for go
//#define sonarSideLeftPin A12
//#define sonarFrontLeftPin A11
//#define sonarFrontCenterPin A10
//#define sonarFrontRightPin A9
//#define sonarSideRightPin A8
#define sonarSideLeftPin 31
#define sonarFrontLeftPin 29
#define sonarFrontCenterPin 27
#define sonarFrontRightPin 25
#define sonarSideRightPin 23
// compass connects to 20/21 (SDA/SCL)
// compass connects to A4/A5

//#define SONAR_DELAY 33
#define SONAR_DELAY 100

//Steering and drive
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700
#define MOTOR_PIN 12
#define STEER_PIN 11
#define STEER_MID 82
#define MOTOR_MID 90
#define MOTOR_SPEED 14 //20 is speedy
#define STEER_AMOUNT 20
#define MOTOR_FORWARD MOTOR_MID-MOTOR_SPEED
#define MOTOR_REVERSE MOTOR_MID+MOTOR_SPEED
#define STEER_LEFT STEER_MID-STEER_AMOUNT
#define STEER_RIGHT STEER_MID+STEER_AMOUNT

Servo motor;
Servo steering;

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
//#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.48 // Declination (degrees) in Sparkfun, Niwot, CO.

long lNumTests;
long maxX, maxY, maxZ, minX, minY, minZ;
bool bCalEnabled;
//int led = 13;
float currentLat, currentLon;
float current_bearing;

#define TEST_POINTS 1000
#define IS_CALIBRATING false

int motorTimer;


/** init_gps() */
//void init_gps() {
//  Serial.print(F("init_gps() ... "));
//  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
//  GPS.begin(9600);
//  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
//  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//  // Set the update rate
//  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
//  // Request updates on antenna status, comment out to keep quiet
//  GPS.sendCommand(PGCMD_ANTENNA);
//
//  // start interrupt timer
//  OCR0A = 0xAF;
//  TIMSK0 |= _BV(OCIE0A);
//
//  Serial.println(F("OK!"));
//}

void init_compass() {
  Serial.println("Initializing LSM9DS1 compass...");
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while(true);
  }

  minX=999999;
  minY=999999;
  minZ=999999;
  maxX=-999999;
  maxY=-999999;
  maxZ=-999999;
  lNumTests = 0;
  bCalEnabled = true;

  float testAdjustX = float(maxX + minX) / 2.0;
  float testAdjustY = float(maxY + minY) / 2.0;

  Serial.print("AdjustX: "); Serial.println(testAdjustX);
  Serial.print("AdjustY: "); Serial.println(testAdjustY);

  Serial.println("Done initializing LSM9DS1 compass");
  
}

float getBearing(float mx, float my) {
  float heading;

//  if (my == 0)
//    heading = (mx < 0) ? 180.0 : 0;
//  else
//    heading = atan2(mx, my);
    
  float adjustX = float(maxX + minX) / 2.0;
  float adjustY = float(maxY + minY) / 2.0;

  float calxMx = mx - float(maxX + minX) / 2.0;
  float calcMy = -my + float(maxY + minY) / 2.0;

  if ((calxMx) == 0)
   heading = (calxMx < 0) ? 180.0 : 0;
  else
   heading = atan2(calxMx, calcMy);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  
  //Serial.print("Raw: "); Serial.print(mx); Serial.print(", "); Serial.print(my); Serial.print(", "); Serial.println(mz);
  //Serial.print("Adjusted: "); Serial.print(mx - adjustX); Serial.print(", "); Serial.print(my - adjustY); Serial.print(", "); Serial.println(mz);
//  Serial.print("Heading: "); Serial.println(heading, 2);
  
  return(heading);

}

///** Office Evolution */
//Location WAYPOINT[] = {
////  Location(-105.1203418,39.9155008),
//  Location(-105.1204598,39.9156695),
//  Location(-105.12072530000002,39.9155944),
//  Location(-105.1209788,39.9155204),
//  Location(-105.1210941,39.9153939),
//  Location(-105.121078,39.9153064),
//  Location(-105.120669,39.9154134)
//};


/* Basketball court */
/*
  Location WAYPOINT[] = {
  Location(-105.08160397410393,39.94177796143009),
  Location(-105.08158653974533,39.94190648894768),
  Location(-105.08174613118172,39.94186741660787)
  };
*/

/* Gizmo Dojo Garage */

//Location WAYPOINT[] = {
//  Location(-104.9800864,39.8869837),
//  Location(-104.9798597,39.8870959),
//  Location(-104.9800046,39.8869426)
//};

//Garage V2
//Location WAYPOINT[] = {
//  Location(-104.98004913, 39.88695907),
//  Location(-104.97996520, 39.88705062),
//  Location(-104.98004913, 39.88694763)
//};

//Race Day!
//Location WAYPOINT[] = {
//  Location(-105.185844, 40.0907034),
//  Location(-105.18517080000001, 40.0912862),
//  Location(-105.1848342, 40.0910574),
//  Location(-105.185844, 40.0907034)
//};

//testing for Race Day!
//Location WAYPOINT[] = {
//  Location(-105.185844, 40.0907034),
//  Location(-105.18517080000001, 40.0912862)
//  Location(-105.1848342, 40.0910574),
//  Location(-105.185844, 40.0907034)
//};

//race day
//Location WAYPOINT[] = {
// Location(-105.185844, 40.0906963),
// Location(-105.185734, 40.0908317),
// Location(-105.1856267, 40.090928100000006),
// Location(-105.1855007, 40.091004),
// Location(-105.1853988, 40.0911251),
// Location(-105.1852834, 40.0912277),
// Location(-105.1851574, 40.0912934),
// Location(-105.1850823, 40.091238),
// Location(-105.1850152, 40.0911969),
// Location(-105.184924, 40.0911395),
// Location(-105.1848516, 40.0910943),
// Location(-105.1848999, 40.0910164),
// Location(-105.1850098, 40.0909343),
// Location(-105.1851064, 40.0908378),
// Location(-105.185203, 40.0907517),
// Location(-105.18531830000002, 40.0906552),
// Location(-105.1854014, 40.0905711)
//};
//race day heat 2
//Location WAYPOINT[] = {
// Location(-105.185820, 40.090692),
// Location(-105.185762, 40.090759),
// Location(-105.185518, 40.090982),
// Location(-105.185365, 40.091115), // Approaching corner 2
// Location(-105.185227, 40.091235),
// Location(-105.185182, 40.091275), // start of corner 2
// Location(-105.185115, 40.091260), // out of corner 2
// Location(-105.184963, 40.091161),
// Location(-105.184858, 40.091088), // Into Corner 3
// Location(-105.184867, 40.091048), // End of corner 3
// Location(-105.185025, 40.090908),
// Location(-105.185322, 40.090638),
// Location(-105.185441, 40.090527), //close to corner 4
// Location(-105.185482, 40.090496), //Into corner 4
// Location(-105.185526, 40.090490), // End corner 4
// Location(-105.185671, 40.090582), // Finish line ish
// Location(-105.185841, 40.090695), // Into corner 1
// Location(-105.186096, 40.090899) // Very overkill past finish
//};
//Location WAYPOINT[] = {
// Location(-105.185820, 40.090692),
// Location(-105.185741, 40.090762), // adj
// Location(-105.185499, 40.090968),  //adj
// Location(-105.185351, 40.091104), // adj Approaching corner 2
// Location(-105.185223, 40.091232), //slightly adj
// Location(-105.185180, 40.091269), // slightly adj start of corner 2
// Location(-105.185115, 40.091260), // out of corner 2
// Location(-105.184963, 40.091161),
// Location(-105.184858, 40.091088), // Into Corner 3
// Location(-105.184853, 40.091041), // adj End of corner 3
// Location(-105.185000, 40.090899), //adj2
// Location(-105.185307, 40.090626), //adj
// Location(-105.185427, 40.090517), // adj close to corner 4
// Location(-105.185471, 40.090488), //adj Into corner 4
// Location(-105.185526, 40.090490), // End corner 4
// Location(-105.185671, 40.090582), // Finish line ish
// Location(-105.185841, 40.090695), // Into corner 1
// Location(-105.186096, 40.090899) // Very overkill past finish
//};
////At Home store, Broomfield
//Location WAYPOINT[] = {
//  Location(-105.087954, 39.920628),
//  Location(-105.088093, 39.920719),
//  Location(-105.088195, 39.920639),
//  Location(-105.088084, 39.920563)
//};

////Erik & Diana's garage, opening to garage, half way down the driveway
//Location WAYPOINT[] = {
//  Location(-104.980635, 39.886556),
//  Location(-104.980518, 39.886533)
//};

////Erik & Diana's garage, half way down the driveway, end of driveway
//Location WAYPOINT[] = {
//  Location(-104.980518, 39.886533),
//  Location(-104.980448, 39.886521)
//};

////Testing At Mark's
//Location WAYPOINT[] = {
//  Location(-105.035242, 39.877564),
//  Location(-105.035129, 39.877436)
//};

//RACE DAY!
Location WAYPOINT[] = {
  Location(-105.185805, 40.090782),
  Location(-105.185706, 40.090868),
  Location(-105.185476, 40.090730),
  Location(-105.185504, 40.090962),
  Location(-105.185326, 40.090849),
  Location(-105.185182, 40.090758),
  Location(-105.185345, 40.090594),
  Location(-105.185414, 40.090606),
  Location(-105.185397, 40.090551),
  Location(-105.185474, 40.090567),
  Location(-105.185805, 40.090782)
};

////MINI RACE!
//Location WAYPOINT[] = {
//  Location(-105.185551, 40.091610),
//  Location(-105.185551, 40.091610),
//  Location(-105.185736, 40.091503)
//};

#define ENABLE_MOTORS

#define MIN_SPEED 150 // should be 60
//#define MAX_SPEED 174 // should be 255
//#define MAX_SPEED 80 // should be 255
#define MAX_DISTANCE 200 // max distance to measure

#define EMERGENCY_STOP_DISTANCE_CM 30 // hit the brakes if any front sensor measures below this value
#define EMERGENCY_STOP_DISTANCE_CM_SIDE 18 //brakes for side

#define MIN_FRONT_DISTANCE    50
#define MIN_SIDE_DISTANCE     20
#define brake_amount_on_turn  80 // 0 to 127
#define brake_delay_on_turn   100 // how long to brake in ms
#define sharp_turn_degrees    45.0f // degrees should be 20
#define ACCURACY              0.00003 //0.00004 used in testing and works well.
#define GPS_PANIC_ACCURACY    0.00005


#define GPS_PERIOD 10

#define ENABLE_SONAR

#define GPS_ADDRESS 0x42

//LSM303 compass;

volatile bool interrupted;

bool forward;
bool pseudoPWMOn;
int speedOnMillis;
int speedOffMillis;
int speedCount;
int reverseMillis;
int leftMillis;
int rightMillis;
int gpsTimer;
int sonarTracker;
int sonarTimer;
bool badSonar[5];
long gpsPanicTimer;
bool waypointChanged;
int debugTimer;
int driveTimer; //necessary to talk to ESC on Helion platform
int motorSpeed;
int steerHeading;

//long millisecondTracker; //testing only

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
/**************************************************************************

   Arduino MEGA 2560

 *************************************************************************/

// Software serial
//PololuQik2s12v10 qik(50, 51, 52);

// Use hardware interupts on Mega (pins 18,19)
//Adafruit_GPS GPS(&Serial1);

TinyGPSPlus gps;
//SoftwareSerial ss(19,18); //NOTE: Not all pins on the Mega and Mega 2560 support change interrupts, so only the following can be used for RX: 10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69).

// HC-SR04 connected to analog pins
NewPing sonar[5] = {
  NewPing(sonarSideLeftPin, sonarSideLeftPin, MAX_DISTANCE), // left
  NewPing(sonarFrontLeftPin, sonarFrontLeftPin, MAX_DISTANCE), // front-left
  NewPing(sonarFrontCenterPin, sonarFrontCenterPin, MAX_DISTANCE), // front-center
  NewPing(sonarFrontRightPin, sonarFrontRightPin, MAX_DISTANCE),   // front-right
  NewPing(sonarSideRightPin, sonarSideRightPin, MAX_DISTANCE)    // right
};

#endif

/**************************************************************************

   Global variables

 *************************************************************************/

//File logger;
Location currentLocation(0, 0);
Location targetLocation(0, 0);
Location previousLocation(0, 0);
unsigned short nextWaypointIndex = 0;
unsigned short waypointCount = sizeof(WAYPOINT) / sizeof(Location);

boolean gpsFix = false;

unsigned int sonar_value[5];

/** init_gps() */
//void init_gps() {
//  Serial.println(F("Initializing GPS. Please wait..."));
//  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
//  GPS.begin(9600);
//  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
//  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//  // Set the update rate
//  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
//  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ); // Added this
//  // Request updates on antenna status, comment out to keep quiet
//  GPS.sendCommand(PGCMD_ANTENNA);
//
//  // start interrupt timer
//  OCR0A = 0xAF;
//  TIMSK0 |= _BV(OCIE0A);
//
//  Serial.println(F("GPS initialization complete!"));
//}

void init_gps() {

  Serial.println(F("Initializing GPS. Please wait..."));
  
  Serial1.begin(9600);

  Serial.println(F("GPS Initialized!"));
  

//  Serial1.println("

//  unsigned int header = 0xB562;
//  unsigned int ID = 0x0608;
//  unsigned int msgSize = 6;
//  unsigned int measRate = 200; //Measurement Rate, GPS measurements are taken every measRate milliseconds
//  unsigned int navRate = 1; //Navigation Rate, in number of measurement cycles. On u-blox 5 and u-blox 6, this parameter cannot be changed, and is always equals 1.
//  unsigned int timeRef = 1; //Alignment to reference time: 0 = UTC time, 1 = GPS time
//  
//  unsigned char msg[11];
//  msg[0] = header>>8;
//  msg[1] = header&0xFF;
//  msg[2] = ID>>8;
//  msg[3] = ID&0xFF;
//  msg[4] = msgSize;
//  msg[5] = measRate>>8;
//  msg[6] = measRate&0xFF;
//  msg[7] = navRate>>8;
//  msg[8] = navRate&0xFF;
//  msg[9] = timeRef>>8;
//  msg[10] = timeRef&0xFF;
//
//  //calculate 8-bit Fletcher Algorithm checksum
//  unsigned char CK_A = 0;
//  unsigned char CK_B = 0;
//  for( int i=0; i<11; i++) {
//    CK_A = CK_A + msg[i]; //THG: checksum calculation wrong
//    CK_B = CK_B + CK_A;
//  }

  //send message
////  for( int i=0; i<11; i++ ) {
////    Serial1.write(msg[i]);
////  }
//  
//  
//  Serial1.write(msg, 11);
//  Serial1.write(CK_A);
//  Serial1.write(CK_B);
//
//  Serial.print("MSG: ");
//  for( int i=0; i<11; i++ ) {
//    Serial.print(msg[i], HEX);
//    Serial.print(" ");
//  }
//  Serial.print(CK_A, HEX);
//  Serial.println(CK_B, HEX);
//
//  while(true) {
//    while( Serial1.available() > 0 ) {
//      Serial.print(Serial1.read());
//    }
//  }

}

//void init_compass() {
//  Serial.println(F("Initializing compass. Please wait... "));
//  Wire.begin();
//
//  compass.init();
//  compass.enableDefault();
//
//  // Calibration values...
//  compass.m_min = (LSM303::vector<int16_t>) {
//    445, -1242, -1005
//  };
//  compass.m_max = (LSM303::vector<int16_t>) {
//    1468, -184, -124
//  };
//
//  Serial.println(F("Compass initialized!"));
//}

double calc_bearing_diff(double current_bearing, double required_bearing) {
  double ret = required_bearing - current_bearing;
  if (ret < -180) {
    ret += 360;
  }
  else if (ret > 180) {
    ret -= 360;
  }
  return ret;
}

float calculate_compass_bearing(double x, double y) {

  // scale up the deltas based on size of lat/long at 40 degrees latitude
  double ax = 53 * (x > 0 ? x : -x);
  double ay = 69 * (y > 0 ? y : -y);

  double radian = 180 / 3.141592;
  if (x > 0) {
    if (y > 0) {
      // 0 through 90
      if (ax > ay) {
        return 90 - radian * atan(ay / ax);
      } else {
        return radian * atan(ax / ay);
      }
    } else {
      if (ax > ay) {
        return 90 + radian * atan(ay / ax);
      } else {
        return 180 - radian * atan(ax / ay);
      }
    }
  } else {
    if (y > 0) {
      if (ax > ay) {
        return 270 + radian * atan(ay / ax);
      } else {
        return 360 - radian * atan(ax / ay);
      }
    } else {
      if (ax > ay) {
        return 270 - radian * atan(ay / ax);
      } else {
        return 180 + radian * atan(ax / ay);
      }
    }
  }
}

float convert_to_decimal_degrees(float f) {
  int d = (int) f / 100;
  f -= d * 100;
  return (f / 60.0) + d;
}

/** calc difference between two DMS (degrees, minutes, seconds) numbers ... not needed if we use decimal degrees everywhere */
double calculate_difference(double l1, double l2) {

  int d1 = l1 / 100;
  int d2 = l2 / 100;
  double m1 = l1 - (d1 * 100);
  double m2 = l2 - (d2 * 100);
  double ret = ((d1 * 60.0f) + m1) - ((d2 * 60.0f) + m2);

  return ret;

}

void get_next_waypoint() {

  if (nextWaypointIndex == waypointCount) {
//    setForwardOff();

    delay(100);
    for( int i=0; i<10; i++ ) {
      steering.write(STEER_MID);
      motor.write(MOTOR_MID);
      delay(20);
    }
    delay(100);
    for( int i=0; i<10; i++ ) {
      steering.write(STEER_MID);
      motor.write(MOTOR_MID);
      delay(20);
    }
    
    while(true);
  }

  targetLocation.latitude  = WAYPOINT[nextWaypointIndex].latitude;
  targetLocation.longitude = WAYPOINT[nextWaypointIndex].longitude;

  nextWaypointIndex++;

  waypointChanged = true;
  gpsPanicTimer = 0;

}

//void set_motor_speeds(int left, int right) {
//
//  setForwardSpeed(MAX_SPEED);
//  if ( left > right ) {
//    turnForXMillis(50, true); //turn left for 50ms
//  } else if ( right > left ) {
//    turnForXMillis(50, false); //turn right for 50ms
//  }
//}

/**
   Get sonar distance in cm. Note that this function does not contain a delay so be sure to delay(33) between calls to avoid false readings.
*/
int ping_sonar(int index) {
  unsigned int uS = sonar[index].ping();
  if (uS == 0) {
    return MAX_DISTANCE;
  }
  sonar_value[index] = uS / US_ROUNDTRIP_CM;
  return sonar_value[index];
}

//void measure_sonar() {
//  // sonar
//  for (int i = 0; i < 5; i++) {
//    sonar_value[i] = ping_sonar(i);
//    delay(SONAR_DELAY);
//  }
//}

/** Hit the brakes if we're about to crash! */
boolean about_to_crash() {
  boolean emergency_stop = false;
  if ( sonar_value[0] < EMERGENCY_STOP_DISTANCE_CM_SIDE ||
       sonar_value[4] < EMERGENCY_STOP_DISTANCE_CM_SIDE ) {
    emergency_stop = true;
  }
  if (sonar_value[1] < EMERGENCY_STOP_DISTANCE_CM ||
      sonar_value[2] < EMERGENCY_STOP_DISTANCE_CM ||
      sonar_value[3] < EMERGENCY_STOP_DISTANCE_CM ) {
    emergency_stop = true;
  }

  return emergency_stop;
}

void avoid_obstacle(boolean turn_left) {

  //brake
//  setForwardOff();
  motorSpeed = MOTOR_MID;

  // hit left or right brakes
  if (turn_left) {
//    setWheelsLeft();
    steerHeading = STEER_LEFT;
  } else {
//    setWheelsRight();
    steerHeading = STEER_RIGHT;
  }

  delay(100);
  for( int i=0; i<10; i++ ) {
    steering.write(steerHeading);
    motor.write(motorSpeed);
    delay(20);
  }
  delay(100);
  for( int i=0; i<10; i++ ) {
    steering.write(steerHeading);
    motor.write(motorSpeed);
    delay(20);
  }

  sonar_value[0] = ping_sonar(0);
  delay(SONAR_DELAY);
  sonar_value[1] = ping_sonar(1);
  delay(SONAR_DELAY);
  sonar_value[2] = ping_sonar(2);
  delay(SONAR_DELAY);
  sonar_value[3] = ping_sonar(3);
  delay(SONAR_DELAY);
  sonar_value[4] = ping_sonar(4);
  delay(SONAR_DELAY);
  // wait
  delay(85);

  // release brakes
//  setForwardSpeed(MAX_SPEED);
  motorSpeed = MOTOR_FORWARD;

  delay(100);
  for( int i=0; i<10; i++ ) {
    steering.write(STEER_MID);
    motor.write(motorSpeed);
    delay(20);
  }
  delay(100);
  for( int i=0; i<10; i++ ) {
    steering.write(STEER_MID);
    motor.write(motorSpeed);
    delay(20);
  }


}

/** Calculate motor speed based on angle of turn. */
//float calculate_motor_speed(float angle_diff_abs) {
//  if (angle_diff_abs > sharp_turn_degrees) {
//    return 0;
//  }
//  else {
//    return (180 - angle_diff_abs) / 180 * MAX_SPEED;
//  }
//}

void setup() {

  pinMode(goPin, INPUT_PULLUP);

//  init_movement();

  Serial.begin(115200);
  Serial.println(F("Welcome to GizmoBot 0.4"));

  gpsTimer = 0;
  waypointChanged = true;
  debugTimer = 0;

  init_compass();
  
  init_gps();

//init motor and servo for steering
  motor.attach(MOTOR_PIN);
  steering.attach(STEER_PIN);
  
  for(int i=0; i<100; i++) //Arming
  {
    motor.write(90);
    delay(20);
  }

//  Serial1.begin(9600); //GPS baud rate


  get_next_waypoint();

  //set up timer2 interrupt
  cli();//stop interrupts

  interrupted = false;

  //set timer0 interrupt at 2kHz
  //  TCCR0A = 0;// set entire TCCR2A register to 0
  //  TCCR0B = 0;// same for TCCR2B
  //  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR3B = 30;// = (16*10^6) / (2000*64) - 1 (must be <256)   THG note: not exactly 1msec, would like to do the math to make it 1ms
  // turn on CTC mode
  //  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  //  TCCR0B |= (1 << CS01) | (1 << CS00);
  // enable timer compare interrupt
  TIMSK3 |= (1 << OCIE3B);

  sei();//allow interrupts

  for ( int i = 0; i < 5; i++ ) {
    sonar_value[i] = 200; //initialize to furthest away
    badSonar[i] = false;
  }

//  //THG testing
//
//  for(int i=0; i<100; i++) {
//    steering.write(STEER_MID + 20);
//    delay(20);
//  }
//  delay(100);
//  
//  for(int i=0; i<100; i++) {
//    steering.write(STEER_MID - 20);
//    delay(20);
//  }
//  delay(100);
//
//  for(int i=0; i<100; i++) {
//    steering.write(STEER_MID);
//    delay(20);
//  }
//
//  delay(100);
//  for( int i=0; i<100; i++ ) {
//    steering.write(STEER_MID);
//    motor.write(MOTOR_REVERSE);
//    delay(20);
//  }
//  delay(100);
//  for( int i=0; i<100; i++ ) {
//    steering.write(STEER_MID);
//    motor.write(MOTOR_MID);
//    delay(20);
//  }
//  delay(5000);//five seconds
//  
//  Serial.println("Done");
//
//  while(true);
//
//  //THG done testing


  Serial.println("Waiting for GPS fix...");

  //  while(!GPS.fix) {
  //    if (GPS.newNMEAreceived()) {
  //      GPS.parse(GPS.lastNMEA());
  //    }
  //  }

  while ( !gps.location.isValid() ) {
    //feed the hungry gps module
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
      //      Serial.println("Got data");
      if ( gps.location.isUpdated() ) {
        //update current location
        currentLocation.latitude = convert_to_decimal_degrees(gps.location.lat()); //rawLat?
        //convert longitude to negative number for WEST
        currentLocation.longitude = 0 - convert_to_decimal_degrees(gps.location.lng()); //rawLng?
        Serial.println("Updated");
      }
    }
  }

  gpsFix = true;

  Serial.println("GPS fixed, waiting for go...");

  while ( digitalRead(goPin) ) { //Wait for go switch
    //feed the hungry gps module
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
      if ( gps.location.isUpdated() ) {
        //update current location
        currentLocation.latitude = convert_to_decimal_degrees(gps.location.lat()); //rawLat?
        //convert longitude to negative number for WEST
        currentLocation.longitude = 0 - convert_to_decimal_degrees(gps.location.lng()); //rawLng?
      }
    }
  }

  Serial.println("GO!");

//  millisecondTracker = 0;
  sonarTimer = 0;
  sonarTracker = 0;
//  sonarTracker = 1;
//  sonarTracker = 2;
//  sonarTracker = 3;
//  sonarTracker = 4;

  gpsPanicTimer = 0;

//  setForwardSpeed(MAX_SPEED);
//  setMovementForward();
  //  delay(5000);

  motorSpeed=MOTOR_FORWARD;
  steerHeading=STEER_MID;

  for( int i=0; i<10; i++ ) {
    steering.write(steerHeading);
    motor.write(motorSpeed);
    delay(20);
  }
  delay(100);
  for( int i=0; i<10; i++ ) {
    steering.write(steerHeading);
    motor.write(motorSpeed);
    delay(20);
  }


}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  //  GPS.read();
}

void loop() {

#define OBSTACLE_AVOID
//#ifdef ENABLE_SONAR
#ifdef OBSTACLE_AVOID
  // obstacle avoidance for front sensors
  if (sonar_value[1] < MIN_FRONT_DISTANCE
      || sonar_value[2] < MIN_FRONT_DISTANCE
      || sonar_value[3] < MIN_FRONT_DISTANCE
     ) {
    // turn left or right depending which sensor has the higher reading
    Serial.println("turn left or right depending which sensor has the higher reading");
    avoid_obstacle(sonar_value[1] > sonar_value[3]);
//    setMovementForward(); //go forward to try to clear the obstacle (WARNING: might get stuck here indefinitely)
//    delay(250);
    for( int i=0; i<10; i++ ) {
      steering.write(STEER_MID);
      motor.write(MOTOR_FORWARD);
      delay(20);
    }
    delay(100);
    for( int i=0; i<10; i++ ) {
      steering.write(STEER_MID);
      motor.write(MOTOR_FORWARD);
      delay(20);
    }
  }

  // check for drift towards wall
  if (sonar_value[0] < MIN_SIDE_DISTANCE
      || sonar_value[4] < MIN_SIDE_DISTANCE) {
    // turn left or right depending which sensor has the higher reading
    Serial.println("turn left or right depending which sensor has the higher reading");
    avoid_obstacle(sonar_value[0] > sonar_value[4]);
//    setMovementForward(); //go forward to try to clear the obstacle (WARNING: might get stuck here indefinitely)
//    delay(250);
    for( int i=0; i<10; i++ ) {
      steering.write(STEER_MID);
      motor.write(MOTOR_FORWARD);
      delay(20);
    }
    delay(100);
    for( int i=0; i<10; i++ ) {
      steering.write(STEER_MID);
      motor.write(MOTOR_FORWARD);
      delay(20);
    }
  }
#endif //OBSTACLE_AVOID
//#endif //ENABLE_SONAR

#define ABOUT_TO_CRASH
#ifdef ABOUT_TO_CRASH
  if (about_to_crash()) {
    Serial.println("About to crash!");
//    setForwardOff();
//    setMovementReverse();
    for( int i=0; i<10; i++ ) {
      steering.write(STEER_MID);
      motor.write(MOTOR_REVERSE);
      delay(20);
    }
    delay(100);
    for( int i=0; i<10; i++ ) {
      steering.write(STEER_MID);
      motor.write(MOTOR_REVERSE);
      delay(20);
    }


//#ifdef DEBUG_SONAR 
    sonar_value[0] = ping_sonar(0);
    delay(SONAR_DELAY);
    sonar_value[1] = ping_sonar(1);
    delay(SONAR_DELAY);
    sonar_value[2] = ping_sonar(2);
    delay(SONAR_DELAY);
//    setReverseOff();
    motor.write(MOTOR_MID);
    sonar_value[3] = ping_sonar(3);
    delay(SONAR_DELAY);
    sonar_value[4] = ping_sonar(4);
    delay(SONAR_DELAY);
//#endif

//    Serial.print("Sonar 9: ");
//    Serial.print(sonar_value[0]);
//    Serial.print(" 11: ");
//    Serial.print(sonar_value[1]);
//    Serial.print(" 12: ");
//    Serial.print(sonar_value[2]);
//    Serial.print(" 1: ");
//    Serial.print(sonar_value[3]);
//    Serial.print(" 3: ");
//    Serial.println(sonar_value[4]);
    
    unsigned int minSonar = 200;
    int minSonarIndex = 2;
    for ( int i = 0; i < 5; i++ ) {
      if ( sonar_value[i] < minSonar && sonar_value[i] != 0 ) {
        minSonar = sonar_value[i];
        minSonarIndex = i;
      }
    }
    if ( minSonarIndex == 0 || minSonarIndex == 1 || minSonarIndex == 2 ) {
      //reverse while turning left for a short time, go forward
      Serial.println("reverse while turning left for a short time, go forward");
//      setWheelsLeft();
//      setMovementReverse();
//      delay(500);
//      setReverseOff();
//      setWheelsRight();
//      setMovementForward();
//      delay(500);
      for( int i=0; i<10; i++ ) {
        steering.write(STEER_LEFT);
        motor.write(MOTOR_REVERSE);
        delay(20);
      }
      delay(100);
      for( int i=0; i<10; i++ ) {
        steering.write(STEER_LEFT);
        motor.write(MOTOR_REVERSE);
        delay(20);
      }
      delay(100);
      for( int i=0; i<10; i++ ) {
        steering.write(STEER_RIGHT);
        motor.write(MOTOR_FORWARD);
        delay(20);
      }
      delay(100);
      for( int i=0; i<10; i++ ) {
        steering.write(STEER_RIGHT);
        motor.write(MOTOR_FORWARD);
        delay(20);
      }

    } else if ( minSonarIndex == 3 || minSonarIndex == 4 ) {
      //reverse while turning right for a short time, go forward
      Serial.println("reverse while turning right for a short time, go forward");
//      setWheelsRight();
//      setMovementReverse();
//      delay(500);
//      setReverseOff();
//      setWheelsLeft();
//      setMovementForward();
//      delay(500);
      for( int i=0; i<10; i++ ) {
        steering.write(STEER_RIGHT);
        motor.write(MOTOR_REVERSE);
        delay(20);
      }
      delay(100);
      for( int i=0; i<10; i++ ) {
        steering.write(STEER_RIGHT);
        motor.write(MOTOR_REVERSE);
        delay(20);
      }
      delay(100);
      for( int i=0; i<10; i++ ) {
        steering.write(STEER_LEFT);
        motor.write(MOTOR_FORWARD);
        delay(20);
      }
      delay(100);
      for( int i=0; i<10; i++ ) {
        steering.write(STEER_LEFT);
        motor.write(MOTOR_FORWARD);
        delay(20);
      }

      //    } else { //center
      //      //reverse while turning to inside track (right at the moment) for a long-ish time, go forward
      //      setWheelsLeft();
      //      setMovementReverse();
      //      delay(500);
    }
    delay(100);
//    setWheelsNotLeft();
//    setWheelsNotRight();
    steering.write(STEER_MID);
    delay(300);
  }
#endif //ABOUT_TO_CRASH

  //1 millisecond elapsed
  if ( interrupted ) {
    interrupted = false;

//#ifdef PLATFORM_NEWBRIGHT
//    //drive logic
//    if ( reverseMillis > 0 ) { //reversing
//      setForwardOff(); //don't drive forward
//      //decrement reversing counter
//      reverseMillis -= 1;
//      setMovementReverse();
//    } else { //not reversing
//      setReverseOff(); //don't drive in reverse
//      if ( pseudoPWMOn ) {
//        //pseudo PWM is high
//        speedCount += 1;
//        if ( speedCount >= speedOnMillis ) {
//          pseudoPWMOn = false;
//          speedCount = 0;
//          setForwardOff();
//        }
//      } else {
//        //pseudo PWM is low
//        speedCount += 1;
//        if ( speedCount >= speedOffMillis ) {
//          pseudoPWMOn = true;
//          speedCount = 0;
//          setMovementForward();
//        }
//      }
//    }
//    if ( leftMillis > 0 ) { //turning left
//      setWheelsNotRight();
//      //decrement left counter
//      leftMillis -= 1;
//      setWheelsLeft();
//    } else {
//      setWheelsNotLeft();
//    }
//    if ( rightMillis > 0 ) { //turning right
//      setWheelsNotLeft();
//      //decrement right counter
//      rightMillis -= 1;
//      setWheelsRight();
//    } else {
//      setWheelsNotRight();
//    }
//#endif //PLATFORM_NEWBRIGHT


    motorTimer += 1;
    if( motorTimer >= 20 ) {
      motorTimer = 0;
      motor.write(motorSpeed);
      steering.write(steerHeading);
    }

    //Other millisecond logic

    sonarTimer += 1;
    if ( sonarTimer >= SONAR_DELAY ) {
      sonarTimer = 0;
      unsigned int sonarChecker = ping_sonar(sonarTracker);
      if ( sonarChecker == 0 ) {
        if ( !badSonar[sonarTracker] ) {
          badSonar[sonarTracker] = true;
        } else {
          sonar_value[sonarTracker] = sonarChecker;
        }
      } else {
        badSonar[sonarTracker] = false;
        sonar_value[sonarTracker] = ping_sonar(sonarTracker);
      }
      //THG trying to fix sonar
      sonarTracker += 1;
      if ( sonarTracker >= 5 ) {
        sonarTracker = 0;
      }
    }

//    millisecondTracker += 1;

    //    if( millisecondTracker % 1000 == 0 ) {
    //      Serial.println(millisecondTracker);
    //    }

    //feed the hungry gps module
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }

    if (gps.location.isUpdated()) {
//      Serial.print("LAT="); Serial.print(gps.location.lat(), 6);
//      Serial.print("       LNG="); Serial.println(gps.location.lng(), 6);

      // update current location
      //currentLocation.latitude = convert_to_decimal_degrees(gps.location.lat());
      currentLocation.latitude = gps.location.lat();
      // convert longitude to negative number for WEST
      //currentLocation.longitude = 0 - convert_to_decimal_degrees(gps.location.lng());
      currentLocation.longitude = 0 - gps.location.lng();


      // get compass heading
//      float cLat, cLon;
//      cLat = gps.location.lat();
//      cLon = gps.location.lng();
//      if ((cLat != currentLat) || (cLon != currentLon)) {
//        currentLat = cLat;
//        currentLon = cLon;
////        Serial.print("LAT="); Serial.print(gps.location.lat(), 6);
////        Serial.print(" LNG="); Serial.println(gps.location.lng(), 6);
//
////        imu.readMag();
////        printHeading(-imu.mx, -imu.my);
////        Serial.println("imu");
//
////  compass.read();
////  float current_bearing  = compass.heading() - 92; // Convert from -Y to +X, minus the chassis offset
////        imu.readMag();
//        current_bearing = getBearing(imu.mx, imu.my);
////        Serial.println("bearing");
//        if (current_bearing < 0) {
//          current_bearing += 360;
//        }
////        Serial.println("adjust");
//      }
    }

//test
        imu.readMag();
//        printHeading(-imu.mx, -imu.my);
//        Serial.println("imu");
        current_bearing = getBearing(imu.mx, imu.my);
        if (current_bearing < 0) {
          current_bearing += 360;
        }


//  compass.read();
//  float current_bearing  = compass.heading() - 92; // Convert from -Y to +X, minus the chassis offset
//        imu.readMag();
        current_bearing = getBearing(imu.mx, imu.my);
//        Serial.println("bearing");
        if (current_bearing < 0) {
          current_bearing += 360;
        }
//        Serial.println("adjust");


    //    if (GPS.newNMEAreceived()) {
    //      if (GPS.parse(GPS.lastNMEA())) {
    //        if (GPS.fix) {
    //          if (!gpsFix) {
    //            gpsFix = true;
    //          }
    //          // update current location
    //          currentLocation.latitude = convert_to_decimal_degrees(GPS.latitude);
    //          // convert longitude to negative number for WEST
    //          currentLocation.longitude = 0 - convert_to_decimal_degrees(GPS.longitude);
    //
    ////          Serial.print("Latitude: ");
    ////          Serial.print(currentLocation.latitude, 8);
    ////          Serial.print("    Longitude: ");
    ////          Serial.println(currentLocation.longitude, 8);
    //
    //        } else {
    //          Serial.println("no GPS");
    //        }
    //      }
    //    }
    //
    //    if (!GPS.fix) {
    //      if (gpsFix) {
    //        gpsFix = false;
    //        // coast
    //        setForwardSpeed(0);
    //        setForwardOff();
    //      }
    //     Serial.println("NO GPS FIX");
    //     return;
    //    }
    //
    //    gpsTimer += 1;
    //
    //    if( gpsTimer > 10 ) {
    //      gpsTimer = 0;
    //
    //      // now that we have a gps location, calculate how to get to the destination
    //      double diffLon = calculate_difference(targetLocation.longitude, currentLocation.longitude);
    //      double diffLat = calculate_difference(targetLocation.latitude, currentLocation.latitude);
    //      float required_compass_bearing = calculate_compass_bearing(diffLon, diffLat);
    //
    //      float angle_diff = calc_bearing_diff(current_bearing, required_compass_bearing);
    //      float angle_diff_abs = fabs(angle_diff);
    //
    //      // have we reached the waypoint yet?
    //      if (fabs(diffLon) <= ACCURACY && fabs(diffLat) <= ACCURACY) {
    ////        Serial.println("Close enough, moving to next point");
    //        get_next_waypoint();
    //        setForwardOff(); //hard code a slight right turn
    //        setWheelsRight();
    //        setMovementForward();
    //        delay(150);
    //        setWheelsNotRight();
    //        setForwardOff();
    //        return;
    //      }
    //
    //      if( angle_diff < -8 ) {
    ////      Serial.println("Trend left");
    //        turnForXMillis(200, true);
    //      } else if( angle_diff > 8 ) {
    ////      Serial.println("Trend right");
    //        turnForXMillis(200, false);
    //      }
    //
    //
    //    }


//this never gets entered, look at it
    if ( gps.location.isUpdated() ) {
//      Serial.println("isUpdated");
      if ( gps.location.isValid() ) {
        if ( !gpsFix ) {
          gpsFix = true;
        }
        //update current location
        currentLocation.latitude = convert_to_decimal_degrees(gps.location.lat()); //rawLat?
        //convert longitude to negative number for WEST
        currentLocation.longitude = 0 - convert_to_decimal_degrees(gps.location.lng()); //rawLng?
        Serial.print("UPLAT: ");
        Serial.print(currentLocation.latitude);
        Serial.print(" UPLNG: ");
        Serial.print(currentLocation.longitude);
        Serial.println("");
      } else {
        Serial.println("no GPS");
      }
    }

    if (!gps.location.isValid()) {
      if (gpsFix) {
        gpsFix = false;
        // coast
//        setForwardSpeed(0);
//        setForwardOff();
          motorSpeed = MOTOR_MID;
      }
      Serial.println("NO GPS FIX");
      //      return;
    }

    gpsTimer += 1;

    if ( gpsTimer > 10 ) {
      gpsTimer = 0;

      // now that we have a gps location, calculate how to get to the destination
      double diffLon = calculate_difference(targetLocation.longitude, currentLocation.longitude);
      double diffLat = calculate_difference(targetLocation.latitude, currentLocation.latitude);

      double distanceKm =
        gps.distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          targetLocation.latitude,
          targetLocation.longitude);
      double courseTo =
        gps.courseTo(
          gps.location.lat(),
          gps.location.lng(),
          targetLocation.latitude,
          targetLocation.longitude);

      Serial.print("Dist: "); Serial.print(distanceKm);
      Serial.print("  Course: "); Serial.println(courseTo);
          
      float required_compass_bearing = calculate_compass_bearing(diffLon, diffLat);
      required_compass_bearing = courseTo;

      float angle_diff = calc_bearing_diff(current_bearing, required_compass_bearing);
      float angle_diff_abs = fabs(angle_diff);

      // have we reached the waypoint yet?
//      if (fabs(diffLon) <= ACCURACY && fabs(diffLat) <= ACCURACY) {
      if( distanceKm <= 2.0 ) {
        Serial.println("Close enough, moving to next point");
        get_next_waypoint();
//        setForwardOff(); //hard code a slight right turn
//        setWheelsRight();
//        setMovementForward();
//        delay(150);
//        setWheelsNotRight();
//        setForwardOff();
        return;
      }

      //THG tweak this for differential steering
      if ( angle_diff < -8 ) {
//        Serial.print("Trend left: ");
//        Serial.println(angle_diff);
//        turnForXMillis(200, true);
        motorSpeed = MOTOR_FORWARD;
        steerHeading = STEER_LEFT;
      } else if ( angle_diff > 8 ) {
//        Serial.print("Trend right: ");
//        Serial.println(angle_diff);
//        turnForXMillis(200, false);
        motorSpeed = MOTOR_FORWARD;
        steerHeading = STEER_RIGHT;

      } else {
        Serial.print("On track: ");
        Serial.println(angle_diff);
      }

    }

    debugTimer += 1;
    if ( debugTimer >= 100 ) {
      debugTimer = 0;

      Serial.print("Lat: ");
      Serial.print(currentLocation.latitude, 8);
      Serial.print("    Long: ");
      Serial.print(currentLocation.longitude, 8);
      Serial.print("    WP: ");
      Serial.print(nextWaypointIndex);

      double diffLon = calculate_difference(targetLocation.longitude, currentLocation.longitude);
      double diffLat = calculate_difference(targetLocation.latitude, currentLocation.latitude);

      Serial.print("  diffLon: ");
      Serial.print(diffLon);
      Serial.print("  diffLat: ");
      Serial.println(diffLat);

//      Serial.print("Sonar 9: ");
//      Serial.print(sonar_value[0]);
//      Serial.print(" 11: ");
//      Serial.print(sonar_value[1]);
//      Serial.print(" 12: ");
//      Serial.print(sonar_value[2]);
//      Serial.print(" 1: ");
//      Serial.print(sonar_value[3]);
//      Serial.print(" 3: ");
//      Serial.println(sonar_value[4]);

    }

    //if 20 seconds elapses and we haven't had a waypoint change, assume we're stuck
    gpsPanicTimer += 1;
    if ( gpsPanicTimer >= 20000 ) {
      gpsPanicTimer = 0;

      if ( !waypointChanged ) {
        Serial.println("GPS Panic, no motion for 20s");
        for( int i=0; i<10; i++ ) {
          steering.write(STEER_MID);
          motor.write(MOTOR_REVERSE);
          delay(20);
        }
        delay(100);
        for( int i=0; i<25; i++ ) {
          steering.write(STEER_MID);
          motor.write(MOTOR_REVERSE);
          delay(20);
        }
        delay(100);
        for( int i=0; i<10; i++ ) {
          steering.write(STEER_RIGHT);
          motor.write(MOTOR_FORWARD);
          delay(20);
        }
        delay(100);
        steering.write(STEER_MID);
        motor.write(MOTOR_FORWARD);

//        waypointChanged = false;
      }
    }
  }
}

ISR(TIMER3_COMPB_vect) { //timer0 interrupt 1kHz
  interrupted = true;
}

