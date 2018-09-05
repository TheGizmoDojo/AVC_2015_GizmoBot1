#include "gizmobot.h"
#include "giz_gps.h"
#include "giz_compass.h"
#include "giz_motor.h"
#include "giz_steering.h"
#include "vec2d.h"
#include "giz_wheel.h"

#define goPin 53 //Flip switch for go

Vec2d current_position_m;//x,y from lat,lng
double current_heading_r;
double desired_heading_r;
double dt,last_micros;
int current_waypoint = 0;
unsigned long navMillis=0;
const uint16_t navPeriod=100;

Vec2d WAYPOINTS_M[] = {
 Vec2d(0,20),
 Vec2d(10,20),
 Vec2d(10,0),
 Vec2d(0,0) 
};

GizCompass giz_compass;
GizGps giz_gps;
GizMotor giz_motor;
GizSteering giz_steering;
GizWheel giz_wheel;

void setup() {

    Serial.begin(115200);
    giz_steering.steer(0);//just to get straight out wheels
    giz_gps.init();//serial not working in constructor
    giz_gps.set_starting_point();


    pinMode(goPin, INPUT_PULLUP);
    delay(1000);
    if (digitalRead(goPin)==LOW) {
      Serial.println("Go switch not ready...");
      while(digitalRead(goPin)==LOW) {};
    }
    delay(500);
    giz_steering.steer(0);
    Serial.println("Waiting for go switch...");
    while(digitalRead(goPin)==HIGH) {};
    giz_steering.steer(0);
    Serial.println("Going Forward!");
    giz_motor.forward();

    
    //testing
//    delay(5000);
//    int steer=0;
//    while (digitalRead(goPin)==LOW) {
//      if (Serial.available()>0) {
//        char incomingByte=Serial.read();
//        if (incomingByte=='a') {
//          steer--;
//        } else if (incomingByte=='s') {
//          steer++;
//        }
//      }
//      giz_steering.steer(0+steer);
//    }
//    Serial.println("Stopping :(5");
//    giz_motor.stop();
//    Serial.print("Steer: ");
//    Serial.println(steer);
//    while(true) {};
    //done testing
    
    //get up to speed before turning because of wheel encoders
//////  giz_wheel.clear_left_encoder();
//////  giz_wheel.clear_right_encoder();

//////  unsigned long driveStraightMillis=millis();
//////  unsigned long updateMillis=driveStraightMillis;
//////  while (driveStraightMillis-millis()<5000) {
//////    if (updateMillis-millis()>=100) {
//////      updateMillis+=100;
//////      giz_wheel.update();
//////      update_current_position(); 
//////      update_current_heading();
//////    }
//////  }

//////  navMillis=millis();
}

void loop() {
  if (navMillis-millis()>=navPeriod) {
    navMillis+=navPeriod;
//    delay(100);
    dt=micros()-last_micros;
    last_micros=micros();

    //get our position && heading
    //giz_compass.update();
    giz_gps.update();//we could run this at slower rate 
    giz_wheel.update();

    //NOTE:  pry want to combine pos,heading update
    update_current_position(); 
    update_current_heading();

    //Serial.println(giz_compass.heading);
    update_desired_heading();
    turn_to_desired_heading();

  Serial.print("position:");
  Serial.print(current_position_m.x,10);
  Serial.print(",");
  Serial.println(current_position_m.y,10);
  Serial.print("heading:");
  Serial.println(current_heading_r);
  Serial.print("desired heading:");
  Serial.println(desired_heading_r);
  Serial.print("wheel right:");
  Serial.print(giz_wheel.rwt_total);
  Serial.print(" left: ");
  Serial.println(giz_wheel.lwt_total);

    //TODO:implement this
    // if(1hz_loop){
    if(did_hit_waypoint()){
        update_to_next_waypoint();
    }
  }
}

static void update_to_next_waypoint(){
    if(current_waypoint >= sizeof(WAYPOINTS_M)/sizeof(WAYPOINTS_M[0])-1){
        giz_motor.stop();
        Serial.println("DONE");
    }else{
        Serial.print("HIT WAYPOINT:");
        Serial.println(current_waypoint + 1);
        current_waypoint++;
    }
}

bool did_hit_waypoint(){

     // GPS can be jittery, and we don't want the car to drive in circles if, say,
     // we're 1.2 meters away, and the GPS updates to say we're 1.2 meters on the
     // other side of the waypoint.
     static double previousDistance_m = 100000;
     static bool close = false;
     const double distance_m = current_position_m.distanceTo(WAYPOINTS_M[current_waypoint]);

     //IE: we were close, but now moving away from waypoint
     if (distance_m < 1 || (close && distance_m > previousDistance_m)) {
        close = false;
        previousDistance_m = 100000;
        return true;
     }

     if (distance_m < 2) {
       close = true;
     }

     previousDistance_m = distance_m;
     return false;
}

//update our desired heading w/ current position and next waypoint 
void update_desired_heading(){
    desired_heading_r=current_position_m.directionTo_r(WAYPOINTS_M[current_waypoint]);
}

//PID control to desired heading
void turn_to_desired_heading(){

    double headingError_r=angle_diff_r(current_heading_r,desired_heading_r);

    double scaleHeadingError=50;//can play w/ this

    giz_steering.steer((int) (headingError_r*scaleHeadingError));

    // TODO: This logic won't work if our heading is e.g. 1 degree and we want to go to 359 degrees
    // if (fabs(headingError_r) < 0.1){
    //   giz_steering.steer(STEER_MID);
    // } else if (headingError_r < 0) {
    //   giz_steering.steer(STEER_LEFT);
    // } else {
    //   giz_steering.steer(STEER_RIGHT);
    // }

}

double angle_diff_r(double x,double y){
    return (double) atan2(sin(x-y), cos(x-y));
}


//combine compass + wheel encoder + gps heading to update position 
void update_current_position(){
    
     //how much complimentary correction from gps to apply(higher the more)
     double gps_correction_amount=0.02;

     //kiss for now just use wheel encoder
     current_position_m.x=(giz_wheel.x_pos_m * (1-gps_correction_amount))
                        + (giz_gps.x_pos_m*gps_correction_amount);
     current_position_m.y=(giz_wheel.y_pos_m * (1-gps_correction_amount))
                        + (giz_gps.y_pos_m*gps_correction_amount);
   
     //I think we want to pass back corrected position to wheel encoder...
     giz_wheel.correct_position(current_position_m);
}

//combine compass + wheel encoder + gps heading to update heading 
void update_current_heading(){

     double gps_correction_amount=0.2;

     current_heading_r=giz_wheel.heading_r;

     //not sure if we can rely on gps heading
    //  current_heading_r=(giz_wheel.heading_r * (1-gps_correction_amount))
   //                     + (giz_gps.heading_r * gps_correction_amount);

  //   giz_wheel.correct_heading(current_heading_r);
}
