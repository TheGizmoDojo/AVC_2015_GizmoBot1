#include "giz_wheel.h"
#include <stdint.h>

const int LEFT_WHEEL_PIN = 2;
const int RIGHT_WHEEL_PIN = 3;
const double WHEEL_SEPARATION_M = .175;
const double WHEEL_RADIUS_M = .100;//change this
const double TICK_RADIUS_M = WHEEL_RADIUS/32;///correct this this

bool GizWheel::_initialized = false;
static uint16_t left_wheel_ticks = 0;
static uint16_t right_wheel_ticks = 0;


static void left_encoder_isr() {
  ++left_wheel_ticks;
}


static void right_encoder_isr() {
  ++right_wheel_ticks;
}


GizWheel::GizWheel(const double wheel_separation_m) :  _wheel_separation_m(WHEEL_SEPARATION_M) {
  if (!_initialized) {
    _initialized = true;
    attachInterrupt(digitalPinToInterrupt(LEFT_WHEEL_PIN), left_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_WHEEL_PIN), right_encoder_isr, CHANGE);
    _initialized = true;
  }
}


GizWheel::GizWheel() : GizWheel(WHEEL_SEPARATION_M) {
}


void update(){

    double lDist=left_wheel_ticks*TICK_RADIUS_M;
    double rDist=right_wheel_ticks*TICK_RADIUS_M;
    //reset ticks
    left_wheel_ticks=0;
    right_wheel_ticks=0;

    if (fabs(lDist - rDist) < 1.0e-6){ 
        x_pos = x_pos + lDist * cos(heading);
        y_pos = y_pos + rDist * sin(heading);
    } else {
        double  turningRadius = WHEEL_SEPARATION_M * (lDist + rDist) / (2 * (rDist - lDist));
        double wd = (rDist - lDist) / WHEEL_SEPARATION_M;

        x_pos = x_pos + turningRadius * sin(wd + heading) - turningRadius * sin(heading);
        y_pos = y_pos - turningRadius * cos(wd + heading) + turningRadius * cos(heading);

        heading = bound_angle(heading + wd);
    }
}

double bound_angle(double a) {
	while(a <= -1*PI) {
		a += 2*PI;
	}
	while(a > PI) {
		a -= 2*PI;
	}
	return a;
}

double GizWheel::get_left_distance_m() {

double GizWheel::get_left_distance_m() {
  const int ticks = left_wheel_ticks;
  left_wheel_ticks -= ticks;
  return ticks * _wheel_separation_m;
}


double GizWheel::get_right_distance_m() {
  const int ticks = right_wheel_ticks;
  right_wheel_ticks -= ticks;
  return ticks * _wheel_separation_m;
}


double GizWheel::peek_left_distance_m() const {
  return left_wheel_ticks * _wheel_separation_m;
}


double GizWheel::peek_right_distance_m() const {
  return right_wheel_ticks * _wheel_separation_m;
}
