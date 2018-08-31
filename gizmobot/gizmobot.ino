#include "gizmobot.h"
#include "giz_gps.h"
#include "giz_compass.h"
#include "giz_motor.h"
#include "giz_steering.h"
#include "vec2d.h"
#include "giz_wheel.h"

//#define goPin 53 //Flip switch for go

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
 Vec2d(0,4),
 Vec2d(2,8)
};




GizCompass giz_compass;
GizGps giz_gps;
GizMotor giz_motor;
GizSteering giz_steering;
GizWheel giz_wheel;

void setup() {

//  pinMode(goPin, INPUT_PULLUP);

Serial.begin(115200);
giz_motor.init();
giz_steering.init();
giz_compass.init();
giz_gps.init();
giz_wheel.init();

delay(1000);
Serial.println("Going Forward!");
giz_motor.forward();

}

void loop() {   
    delay(100);
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

    Serial.print("position:");
    Serial.print(current_position.x,10);
    Serial.print(",");
    Serial.println(current_position.y,10);

    Serial.print("heading:");
    Serial.println(current_heading);

    //TODO:implement this
    // if(1hz_loop){
    if(did_hit_waypoint()){
        update_to_next_waypoint();
    }
   
// }

}

static void update_to_next_waypoint(){
    if(current_waypoint >= sizeof(WAYPOINTS)/sizeof(WAYPOINTS[0])){
        giz_motor.stop();
    }else{
        current_waypoint++;
    }
}

bool did_hit_waypoint(){
     if(current_position.distance(WAYPOINTS[current_waypoint]) < 1){
        return true;
     }
     return false;
}

//update our desired heading w/ current position and next waypoint 
void update_desired_heading(){
   desired_heading=current_position.directionTo(WAYPOINTS[current_waypoint]);

    Serial.print("desired heading:");
    Serial.println(desired_heading);

}

//PID control to desired heading
void turn_to_desired_heading(){
    double headingError=angle_diff(current_heading,desired_heading);
    double scaleHeadingError=10;//can play w/ this
    giz_steering.steer((int) (headingError*scaleHeadingError));
}

double angle_diff(double x,double y){
    return (double) atan2(sin(x-y), cos(x-y));
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

