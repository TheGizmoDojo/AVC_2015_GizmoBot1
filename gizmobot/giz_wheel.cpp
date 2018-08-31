#include "giz_wheel.h"
#include <stdint.h>
#include <Arduino.h>

const int LEFT_WHEEL_PIN = 2;
const int RIGHT_WHEEL_PIN = 3;
const double WHEEL_SEPARATION_M = .175;
const double WHEEL_RADIUS_M = .100;//change this
const double TICK_RADIUS_M = WHEEL_RADIUS_M / 32;///correct this this

bool GizWheel::_initialized = false;
static uint16_t left_wheel_ticks = 0;
static uint16_t right_wheel_ticks = 0;


static void left_encoder_isr() {
  ++left_wheel_ticks;
}


static void right_encoder_isr() {
  ++right_wheel_ticks;
}


GizWheel::GizWheel(const double wheel_separation_m) : _wheel_separation_m(WHEEL_SEPARATION_M) {
  if (!_initialized) {
    _initialized = true;
    attachInterrupt(digitalPinToInterrupt(LEFT_WHEEL_PIN), left_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_WHEEL_PIN), right_encoder_isr, CHANGE);
  }
}


GizWheel::GizWheel() : GizWheel(WHEEL_SEPARATION_M) {
}

static double bound_angle_r(double a) {
  while(a <= -M_PI) {
    a += 2 * M_PI;
  }
  while(a > M_PI) {
    a -= 2 * PI;
  }
  return a;
}

void GizWheel::update() {

    double lDist_m=left_wheel_ticks*TICK_RADIUS_M;
    double rDist_m=right_wheel_ticks*TICK_RADIUS_M;
    //reset ticks
    left_wheel_ticks=0;
    right_wheel_ticks=0;

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
