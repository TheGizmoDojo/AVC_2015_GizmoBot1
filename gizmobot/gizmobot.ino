#include "gizmobot.h"
#include "giz_gps.h"
#include "giz_compass.h"
#include "giz_motor.h"
#include "giz_steering.h"
#include "vec2d.h"
#include "giz_wheel.h"

Vec2d current_position;//x,y from lat,lng
double current_heading;
double desired_heading;
double dt,last_micros;
int current_waypoint = 0;


//   Vec2d WAYPOINTS[] = {
//     Vec2d(-104.98004913, 39.88695907),
//     Vec2d(-104.97996520, 39.88705062),
//     Vec2d(-104.98004913, 39.88694763)
//   };

Vec2d WAYPOINTS[] = {
 Vec2d(0,1),
 Vec2d(2,2)
};




GizCompass giz_compass;
GizGps giz_gps;
GizMotor giz_motor;
GizSteering giz_steering;
GizWheel giz_wheel;

void setup() {
Serial.begin(115200);
giz_motor.init();
giz_steering.init();
giz_compass.init();
giz_gps.init();
giz_wheel.init();

//init wheel encoder 
//setup distance sensors
}

void loop() {
    dt=micros()-last_micros;
    last_micros=micros();

      //get our position && heading
      //giz_compass.update();
      //giz_gps.update();//we could run this at slower rate 

     giz_wheel.update();

     //NOTE:  pry want to combine pos,heading update
     update_current_position(); 
     update_current_heading();
    //Serial.println(giz_compass.heading);
     update_desired_heading();
     turn_to_desired_heading();

    //TODO:implement this
    // if(1hz_loop){
    if(did_hit_waypoint()){
        update_to_next_waypoint();
    }
    //    if(did_finish()){
    //        stop();
    //    }
    // }

}

static void update_to_next_waypoint(){
    current_waypoint++;
}

bool did_hit_waypoint(){
     if(current_waypoint.distanceTo(WAYPOINTS[current_waypoint]) < 1){
        return true;
     }
     return false;
}

//update our desired heading w/ current position and next waypoint 
void update_desired_heading(){
       desired_heading=current_position.directionTo(WAYPOINTS[current_waypoint]);
}

void turn_to_desired_heading(){
    //PID control to desired heading
    //for now just KISS
    double headingError= desired_heading-current_heading;

    if (fabs(headingError) < 0.1){
      giz_steering.steer(STEER_MID);
    } else if (headingError < 0) {
      giz_steering.steer(STEER_LEFT);
    } else {
      giz_steering.steer(STEER_RIGHT);
    }
}

//combine compass + wheel encoder + gps heading to update position 
void update_current_position(){
     //kiss for now just use wheel encoder
     current_position.x=giz_wheel.x_pos;
     current_position.y=giz_wheel.y_pos;
}

//combine compass + wheel encoder + gps heading to update heading 
void update_current_heading(){
     current_heading=giz_wheel.heading;
}

