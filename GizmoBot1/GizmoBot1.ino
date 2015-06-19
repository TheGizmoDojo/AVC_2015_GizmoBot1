/** 
 * Arduino code for an autonomous vehicle.
 *
 * Author: Andy Grove
 *
 * License: None. Feel free to use in your projects.
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <NewPing.h>
#include <LSM303.h>
 
#include "TGD_Movement.h" // Set pin assignments in "TGD_Movement.h"
#include "location.h"

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


/** Office Evolution */
Location WAYPOINT[] = {
//  Location(-105.1203418,39.9155008),
  Location(-105.1204598,39.9156695),
  Location(-105.12072530000002,39.9155944),
  Location(-105.1209788,39.9155204),
  Location(-105.1210941,39.9153939),
  Location(-105.121078,39.9153064),
  Location(-105.120669,39.9154134)
};


/* Basketball court */
/*
Location WAYPOINT[] = {
  Location(-105.08160397410393,39.94177796143009),
  Location(-105.08158653974533,39.94190648894768),
  Location(-105.08174613118172,39.94186741660787)
};
*/

/* Gizmo Dojo Garage */
/*
Location WAYPOINT[] = {
  Location(-104.9800864,39.8869837),
  Location(-104.9798597,39.8870959),
  Location(-104.9800046,39.8869426)
};
*/

#define ENABLE_MOTORS

#define MIN_SPEED 150 // should be 60
#define MAX_SPEED 174 // should be 255
#define MAX_DISTANCE 200 // max distance to measure

#define EMERGENCY_STOP_DISTANCE_CM 20 // hit the brakes if any front sensor measures below this value

#define MIN_FRONT_DISTANCE    50
#define MIN_SIDE_DISTANCE     20
#define brake_amount_on_turn  80 // 0 to 127
#define brake_delay_on_turn  100 // how long to brake in ms
#define sharp_turn_degrees    45.0f // degrees should be 20
#define ACCURACY            0.00004 //0.00004 used in testing and works well.

#define goPin 10 //Flip switch for go

#define GPS_PERIOD 10

LSM303 compass;

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

long millisecondTracker; //testing only

// compass connects to A4/A5

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
/**************************************************************************
 *
 * Arduino MEGA 2560
 *
 *************************************************************************/

// Software serial
//PololuQik2s12v10 qik(50, 51, 52);

// Use hardware interupts on Mega (pins 18,19)
Adafruit_GPS GPS(&Serial1); 

// HC-SR04 connected to analog pins
NewPing sonar[5] = {
  NewPing(A12, A12, MAX_DISTANCE), // left
  NewPing(A11, A11, MAX_DISTANCE), // front-left
  NewPing(A10, A10, MAX_DISTANCE), // front-center
  NewPing(A9, A9, MAX_DISTANCE),   // front-right
  NewPing(A8, A8, MAX_DISTANCE)    // right
};

// compass connects to 20/21 (SDA/SCL)

#endif

/**************************************************************************
 *
 * Global variables
 *
 *************************************************************************/

//File logger;
Location currentLocation(0,0);
Location targetLocation(0,0);
unsigned short nextWaypointIndex = 0;
unsigned short waypointCount = sizeof(WAYPOINT) / sizeof(Location);

boolean gpsFix = true;

unsigned int sonar_value[5];

/** init_gps() */
void init_gps() {
  Serial.println(F("Initializing GPS. Please wait..."));
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ); // Added this
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // start interrupt timer
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  
  Serial.println(F("GPS initialization complete!"));
}

void init_compass() {
  Serial.println(F("Initializing compass. Please wait... "));
  Wire.begin();
  
  compass.init();
  compass.enableDefault();
  
  // Calibration values...
  compass.m_min = (LSM303::vector<int16_t>){445, -1242, -1005};
  compass.m_max = (LSM303::vector<int16_t>){1468, -184, -124};

  Serial.println(F("Compass initialized!"));
}


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
  double ax = x>0 ? x : -x;
  double ay = y>0 ? y : -y;
  double radian = 180 / 3.141592;
  if (x>0) {
    if (y>0) {
      // 0 through 90
      if (ax>ay) {
        return 90 - radian * atan(ay/ax);
      }
      else {
        return radian * atan(ax/ay);
      }
    }
    else {
      if (ax>ay) {
        return 90 + radian * atan(ay/ax);
      }
      else {
        return 180 - radian * atan(ax/ay);
      }
    }
  }
  else {
    if (y>0) {
      if (ax>ay) {
        return 270 + radian * atan(ay/ax);
      }
      else {
        return 360 - radian * atan(ax/ay);
      }
    }
    else {
      if (ax>ay) {
        return 270 - radian * atan(ay/ax);
      }
      else {
        return 180 + radian * atan(ax/ay);
      }
    }
  }
}

float convert_to_decimal_degrees(float f) {
  int d = (int) f/100;
  f -= d*100;
  return (f/60.0) + d;
}




/** calc difference between two DMS (degrees, minutes, seconds) numbers ... not needed if we use decimal degrees everywhere */
double calculate_difference(double l1, double l2) {

  int d1 = l1 / 100;
  int d2 = l2 / 100;
  double m1 = l1 - (d1 * 100);
  double m2 = l2 - (d2 * 100);
  double ret = ((d1*60.0f)+m1) - ((d2*60.0f)+m2);

  return ret;
   
}


void get_next_waypoint() {

  if (nextWaypointIndex == waypointCount) {
    set_motor_speeds(0,0);
    while (1) {
      delay(1000);
    }
  }
  
  targetLocation.latitude  = WAYPOINT[nextWaypointIndex].latitude;
  targetLocation.longitude = WAYPOINT[nextWaypointIndex].longitude;
  
  
  nextWaypointIndex++;
} 

void set_motor_speeds(int left, int right) {
  

//#ifdef ENABLE_MOTORS
//  // I wired the motors backwards in every possible way ... so reverse polarity *and* left/right here
//#endif

  setForwardSpeed(MAX_SPEED);
  if( left > right ) {
    turnForXMillis(50, true); //turn left for 50ms
  } else if( right > left ) {
    turnForXMillis(50, false); //turn right for 50ms
  }


}

/** 
 * Get sonar distance in cm. Note that this function does not contain a delay so be sure to delay(33) between calls to avoid false readings. 
 */
int ping_sonar(int index) {
  unsigned int uS = sonar[index].ping();
  if (uS==0) {
    return MAX_DISTANCE;
  }
  sonar_value[index] = uS / US_ROUNDTRIP_CM;
  return sonar_value[index];
}

void measure_sonar() {
  // sonar
//  Serial.print(F("SONAR:"));
  for (int i=0; i<5; i++) {
    sonar_value[i] = ping_sonar(i);
//    Serial.print(F("  "));
//    Serial.print(sonar_value[i]);
    delay(33);
  }
//  Serial.println();
  
  // record sensor data
}  

/** Hit the brakes if we're about to crash! */
boolean about_to_crash() {
  boolean emergency_stop = false;
  for (int i=0; i<5; i++) {
    if (sonar_value[i] < EMERGENCY_STOP_DISTANCE_CM) {
      emergency_stop = true;
      break;
    }
  }
  if (emergency_stop) {
//    delay(100);
    reverseForXMillis(50); //reverse for 50ms
  }
  return emergency_stop;
}  

void avoid_obstacle(boolean turn_left) {

  //brake
  reverseForXMillis(50); //brake for 50ms
  setForwardSpeed(0);

  // hit left or right brakes  
  if (turn_left) {
    turnForXMillis(250, true); //turn left for 250ms
  } else {
    turnForXMillis(250, false); //turn right for 250ms
  }
  
  // wait
//  delay(brake_delay_on_turn);
  delay(250);

  // release brakes

  // which sonar to monitor?  
  //Note: the 2 lines setting front_index and side_index are equivalent to the following:
//  if( turn_left ) {
//    front_index = 3;
//    side_index = 4;
//  } else {
//    front_index = 1;
//    side_index = 0;
//  }
  int front_index = turn_left ? 3 : 1;
  int side_index = turn_left ? 4 : 0;
  
  int count = 0;
  while (sonar_value[2] < MIN_FRONT_DISTANCE 
    || sonar_value[front_index] < MIN_FRONT_DISTANCE
    || sonar_value[side_index] < MIN_SIDE_DISTANCE) {
      
    measure_sonar();
    if (!about_to_crash()) {
      if (turn_left) {
//        set_motor_speeds(0, MAX_SPEED);
        setForwardSpeed(MAX_SPEED);
        turnForXMillis(250, true); //turn left for 250ms
      } else {
//        set_motor_speeds(MAX_SPEED, 0);
        setForwardSpeed(MAX_SPEED);
        turnForXMillis(250, false); //turn right for 250ms
      }
    }
  }
  
  // coast briefly
  set_motor_speeds(0,0);
}

/** Calculate motor speed based on angle of turn. */
float calculate_motor_speed(float angle_diff_abs) {
  if (angle_diff_abs > sharp_turn_degrees) {
    return 0;
  }
  else {
    return (180 - angle_diff_abs) / 180 * MAX_SPEED;
  }
}  

void setup() {
  
  pinMode(goPin, INPUT);
  
  Serial.begin(115200);
  Serial.println(F("Welcome to GizmoBot 0.2"));
//  Serial.println(F("Our goal is to ensure your satisfaction with our products..."));
//  Serial.println(F("If at any time you are unsatisfied with the performance of our products,"));
//  delay(1500);
//  Serial.println(F("it's probably your fault."));
  
  gpsTimer = 0;
  
  init_compass();
  init_gps();
  
  
  get_next_waypoint();
  
  // start switch
  //pinMode(40, INPUT);
  
  
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

  Serial.println("Ready...");
  
  while( digitalRead(goPin) ); //Wait for go switch
  
  Serial.println("GO!");
  
  millisecondTracker = 0;
  
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  GPS.read();
}

/*
void loop_test_compass() {
  HMC6352.Wake();
  float current_bearing  = HMC6352.GetHeading();
  HMC6352.Sleep();
  Serial.println(current_bearing);
  delay(1000);
}
*/

/*
void loop_test_motors() {
  set_motor_speeds(60,0);
  delay(500);
  set_motor_speeds(0,0);
  delay(1000);
}
*/

void loop() {

//  Serial.println(F("Looping. Please pay attention to my damn output!"));
 
  // check for new GPS info 
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {
      if (GPS.fix) {
        if (!gpsFix) {
          gpsFix = true;
        }
        // update current location
        currentLocation.latitude = convert_to_decimal_degrees(GPS.latitude);
        // convert longitude to negative number for WEST
        currentLocation.longitude = 0 - convert_to_decimal_degrees(GPS.longitude); 
        
        Serial.print("Latitude: ");
        Serial.print(currentLocation.latitude, 8);
        Serial.print("    Longitude: ");
        Serial.println(currentLocation.longitude, 8);
        
      }
    }  
  }
  
  if (!GPS.fix) {
    if (gpsFix) {
      gpsFix = false;
      // coast
      set_motor_speeds(0, 0);
    }
   Serial.println("NO GPS FIX");
   return;
  }
  
  measure_sonar();
  
  // obstacle avoidance for front sensors
  if (sonar_value[1] < MIN_FRONT_DISTANCE 
    || sonar_value[2] < MIN_FRONT_DISTANCE 
    || sonar_value[3] < MIN_FRONT_DISTANCE 
    ) {
    // turn left or right depending which sensor has the higher reading
    avoid_obstacle(sonar_value[1] > sonar_value[3]);
  }

  // check for drift towards wall
  if (sonar_value[0] < MIN_SIDE_DISTANCE 
    || sonar_value[4] < MIN_SIDE_DISTANCE) {
      
    // turn left or right depending which sensor has the higher reading
    avoid_obstacle(sonar_value[0] > sonar_value[4]);
  }
  
  if (about_to_crash()) {
    return;
  }
  
  // get compass heading
//  HMC6352.Wake();
  compass.read();
  float current_bearing  = compass.heading() - 92; // Convert from -Y to +X, minus the chassis offset
  if (current_bearing < 0) {
    current_bearing += 360;
  }
  Serial.print(F("Compass Heading: "));
  Serial.println(current_bearing);
  Serial.println(" ");
//  HMC6352.Sleep();

  // now that we have a gps location, calculate how to get to the destination
  double diffLon = calculate_difference(targetLocation.longitude, currentLocation.longitude); 
  double diffLat = calculate_difference(targetLocation.latitude, currentLocation.latitude); 
  float required_bearing = calculate_compass_bearing(diffLon, diffLat);
  
  float angle_diff = calc_bearing_diff(current_bearing, required_bearing);
  float angle_diff_abs = fabs(angle_diff);
  
  // have we reached the waypoint yet?
  if (fabs(diffLon) <= ACCURACY && fabs(diffLat) <= ACCURACY) {
    get_next_waypoint();
    return;
  }

  // determine new motor speeds
  int left_speed = MAX_SPEED;
  int right_speed = MAX_SPEED;

  if (angle_diff < 0) {
    left_speed = calculate_motor_speed(angle_diff_abs);
  }
  else if (angle_diff > 0) {
    right_speed = calculate_motor_speed(angle_diff_abs);
  }

  // set speed according to navigation
//  set_motor_speeds(left_speed, right_speed);  
  
    if( interrupted ) {
    interrupted = false;
  
    //drive logic
    if( reverseMillis > 0 ) { //reversing
      setForwardOff(); //don't drive forward
      //decrement reversing counter
      reverseMillis -= 1;
      setMovementReverse();
    } else { //not reversing
      setReverseOff(); //don't drive in reverse
      if( pseudoPWMOn ) {
        //pseudo PWM is high
        speedCount += 1;
        if( speedCount >= speedOnMillis ) {
          Serial.print("PWM off");
//          Serial.print(speedCount);
//          Serial.print(speedOnMillis);
          pseudoPWMOn = false;
          speedCount = 0;
          setForwardOff();
        }
      } else {
        //pseudo PWM is low
        speedCount += 1;
        if( speedCount >= speedOffMillis ) {
          Serial.print("PWM on");
//          Serial.print(speedCount);
//          Serial.print(speedOffMillis);
          pseudoPWMOn = true;
          speedCount = 0;
          setMovementForward();
        }
      }
    }
    if( leftMillis > 0 ) { //turning left
      setWheelsNotRight();
      //decrement left counter
      leftMillis -= 1;
      setWheelsLeft();
    } else {
      setWheelsNotLeft();
    }
    if( rightMillis > 0 ) { //turning right
      setWheelsNotLeft();
      //decrement right counter
      rightMillis -= 1;
      setWheelsRight();
    } else {
      setWheelsNotRight();
    }
  
    //Other millisecond logic
    
    millisecondTracker += 1;
    
    if( millisecondTracker % 1000 == 0 ) {
      Serial.println(millisecondTracker);
    }
    
    gpsTimer += 1;
    
    if( gpsTimer 
  
  }

  
}

ISR(TIMER3_COMPB_vect){//timer0 interrupt 1kHz
  interrupted = true;
}

