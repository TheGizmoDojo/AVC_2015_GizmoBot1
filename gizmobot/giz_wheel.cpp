#include "giz_wheel.h"
#include <stdint.h>
#include <Arduino.h>

//at speed 20=380 ticks/second
//at 10=170 ticks/second
//at 7=95 ticks/second
//at 6=71 ticks/second
//at 5=44 ticks/second

const int LEFT_WHEEL_PIN = 2;
const int RIGHT_WHEEL_PIN = 3;
const double WHEEL_SEPARATION_M = .175;
const double WHEEL_DIAMETER = 0.079;
const double WHEEL_CIRCUMFERENCE =WHEEL_DIAMETER * PI;
const double TICk_DISTANCE_M = WHEEL_CIRCUMFERENCE/14;


bool GizWheel::_initialized = false;
static uint16_t left_wheel_ticks = 0;
static uint16_t right_wheel_ticks = 0;

double last_left_encoder_update;
double last_right_encoder_update;

static void left_encoder_isr() {
  if(micros()-last_left_encoder_update > 12000){
      left_wheel_ticks++;
      last_left_encoder_update=micros();//hmm,  maybe outside if?
  }
}

static void right_encoder_isr() {
  if(micros()-last_right_encoder_update > 12000){
      right_wheel_ticks++;
      last_right_encoder_update=micros();
  }
}

GizWheel::GizWheel(){
  if (!_initialized) {
    _initialized = true;
    attachInterrupt(digitalPinToInterrupt(LEFT_WHEEL_PIN), left_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_WHEEL_PIN), right_encoder_isr, CHANGE);
  }
}


double lwt_total=0;
double rwt_total=0;


void GizWheel::update() {

 Serial.print("l:");
 Serial.println(left_wheel_ticks);

 Serial.print("r:");
 Serial.println(right_wheel_ticks); 


    //we sometime get bad data from wheel necoder
    //it will just spike, lets try an ignore spikes
//// if(left_wheel_ticks - right_wheel_ticks > 15){
////	 left_wheel_ticks=right_wheel_ticks;
//// }
//// if(right_wheel_ticks - left_wheel_ticks > 15){
////	 right_wheel_ticks=left_wheel_ticks;
//// }

    double lwt=left_wheel_ticks;
    double rwt=right_wheel_ticks;
    //set back to 0(or close)
    left_wheel_ticks-=lwt;
    right_wheel_ticks-=rwt;

    lwt_total+=lwt;
    rwt_total+=rwt;

    double lDist_m=lwt*TICk_DISTANCE_M;
    double rDist_m=rwt*TICk_DISTANCE_M;

    if (fabs(lDist_m - rDist_m) < 1.0e-6){ 
        x_pos_m = x_pos_m + lDist_m * cos(heading_r);
        y_pos_m = y_pos_m + rDist_m * sin(heading_r);
    } else {
        double turningRadius_m = WHEEL_SEPARATION_M * (lDist_m + rDist_m) / (2 * (rDist_m - lDist_m));
        double wd = (rDist_m - lDist_m) / WHEEL_SEPARATION_M;

        x_pos_m = x_pos_m + turningRadius_m * sin(wd + heading_r) - turningRadius_m * sin(heading_r);
        y_pos_m = y_pos_m - turningRadius_m * cos(wd + heading_r) + turningRadius_m * cos(heading_r);

        heading_r = bound_angle_r(heading_r + wd);
    }
}

double GizWheel::bound_angle_r(double a) {
  while(a <= -M_PI) {
    a += 2 * M_PI;
  }
  while(a > M_PI) {
    a -= 2 * PI;
  }
  return a;
}

double GizWheel::get_left_distance_m() {
  const int ticks = left_wheel_ticks;
  left_wheel_ticks -= ticks;
  return ticks * WHEEL_SEPARATION_M;
}


double GizWheel::get_right_distance_m() {
  const int ticks = right_wheel_ticks;
  right_wheel_ticks -= ticks;
  return ticks * WHEEL_SEPARATION_M;
}


double GizWheel::peek_left_distance_m() const {
  return left_wheel_ticks * WHEEL_SEPARATION_M;
}


double GizWheel::peek_right_distance_m() const {
  return right_wheel_ticks * WHEEL_SEPARATION_M;
}
