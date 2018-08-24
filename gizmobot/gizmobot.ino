#include "gizmobot.h"
#include "giz_gps.h"
#include "giz_compass.h"
#include "giz_motor.h"
#include "giz_steering.h"
#include "vec2d.h"
//#include "giz_wheel_encoder.h"

float position;//x,y from lat,lng
float current_heading;
float desired_heading;
double dt,last_micros;


 Vec2d WAYPOINTS[] = {
   Vec2d(-104.98004913, 39.88695907),
   Vec2d(-104.97996520, 39.88705062),
   Vec2d(-104.98004913, 39.88694763)
 };

GizCompass giz_compass;
GizGps giz_gps;
GizMotor giz_motor;
GizSteering giz_steering;

void setup() {
Serial.begin(115200);
giz_motor.init();
giz_steering.init();
giz_compass.init();
giz_gps.init();

//init wheel encoder 
//setup distance sensors
}

void loop() {
    dt=micros()-last_micros;
    last_micros=micros();

      //get our position && heading
      giz_compass.update();
      giz_gps.update();//we could run this at slower rate 

      //wheel_encoder.update();

    //update_position(); 
    //Serial.println(giz_compass.heading);
    //update_current_heading();
    //update_desired_heading();
    //turn_to_desired_heading();

    //TODO:implement this
    // if(1hz_loop){
    //    if(did_hit_waypoint()){
    //        update_to_next_waypoint();
    //    }
    //    if(did_finish()){
    //        stop();
    //    }
    // }

}

static void increment_desired_waypoint(){


}

bool did_hit_waypoint(){

}


//update our desired heading w/ current position and next waypoint 
void update_desired_heading(){

}

void turn_to_desired_heading(){
    //PID control to desired heading
}


//combine compass + wheel encoder + gps heading to update position 
void update_position(){
    
}

//combine compass + wheel encoder + gps heading to update heading 
void update_current_heading(){

}

