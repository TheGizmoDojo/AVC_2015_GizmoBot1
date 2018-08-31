#include "gizmobot.h"
#include "giz_gps.h"
#include "giz_compass.h"
#include "giz_motor.h"
#include "giz_steering.h"
#include "vec2d.h"
#include "giz_wheel.h"

Vec2d current_position_m;//x,y from lat,lng
double current_heading_r;
double desired_heading_r;
double dt,last_micros;
int current_waypoint = 0;


//   Vec2d WAYPOINTS[] = {
//     Vec2d(-104.98004913, 39.88695907),
//     Vec2d(-104.97996520, 39.88705062),
//     Vec2d(-104.98004913, 39.88694763)
//   };

Vec2d WAYPOINTS_M[] = {
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
     // GPS can be jittery, and we don't want the car to drive in circles if, say,
     // we're 1.2 meters away, and the GPS updates to say we're 1.2 meters on the
     // other side of the waypoint.
     static double previousDistance_m = 100000;
     static bool close = false;

     const double distance_m = current_position_m.distanceTo(WAYPOINTS_M[current_waypoint]);
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

void turn_to_desired_heading(){
    //PID control to desired heading
    //for now just KISS
    double headingError_r= desired_heading_r-current_heading_r;

    // TODO: This logic won't work if our heading is e.g. 1 degree and we want to go to 359 degrees
    if (fabs(headingError_r) < 0.1){
      giz_steering.steer(STEER_MID);
    } else if (headingError_r < 0) {
      giz_steering.steer(STEER_LEFT);
    } else {
      giz_steering.steer(STEER_RIGHT);
    }
}

//combine compass + wheel encoder + gps heading to update position 
void update_current_position(){
     //kiss for now just use wheel encoder
     current_position_m.x=giz_wheel.x_pos_m;
     current_position_m.y=giz_wheel.y_pos_m;
}

//combine compass + wheel encoder + gps heading to update heading 
void update_current_heading(){
     current_heading_r=giz_wheel.heading_r;
}

