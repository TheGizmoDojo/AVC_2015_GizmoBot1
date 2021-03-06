#include "giz_wheel.h"

//at speed 20=380 ticks/second
//at 10=170 ticks/second
//at 7=95 ticks/second
//at 6=71 ticks/second
//at 5=44 ticks/second

const int LEFT_WHEEL_PIN = 2;
const int RIGHT_WHEEL_PIN = 3;
const double WHEEL_SEPARATION_M = .175;
const double WHEEL_DIAMETER_M = 0.079;
const double WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * PI;
const int TICKS_PER_REVOLUTION = 26;
// These values from observation - being commanded to drive 20m and measuring 18.69m
const double ADJUSTMENT = 18.69/20.0*19.74/19.44*19.41/19.55;
const double TICK_DISTANCE_M = WHEEL_CIRCUMFERENCE_M/TICKS_PER_REVOLUTION/ADJUSTMENT;
//const int ISR_CUTOFF_US = 12000;
const int ISR_CUTOFF_US = 1850; //for speed 9
bool pin_8_state=LOW; //testing (TP17)
bool pin_9_state=LOW; //testing (TP18)
const double DRIFT_CORRECTION_FACTOR = 1.0025;

bool GizWheel::_initialized = false;
static uint16_t left_wheel_ticks=0;
static uint16_t right_wheel_ticks=0;

static long last_left_encoder_update = 0;
static long last_right_encoder_update = 0;

static void left_encoder_isr() {
  const long interruptMicros=micros();
  if(interruptMicros-last_left_encoder_update > ISR_CUTOFF_US) {
    left_wheel_ticks++;
    last_left_encoder_update=interruptMicros; //hmm,  maybe outside if?
  }
//  left_wheel_ticks++;
}

static void right_encoder_isr() {
  digitalWrite(8, pin_8_state);
  pin_8_state ^= 1; //toggle
  const unsigned long interruptMicros=micros();
  if(interruptMicros-last_right_encoder_update > ISR_CUTOFF_US) {
    digitalWrite(9, pin_9_state);
    pin_9_state ^= 1; //toggle
    right_wheel_ticks++;
    last_right_encoder_update=interruptMicros;
  }
//  right_wheel_ticks++;
}

void GizWheel::clear_left_encoder() {
  left_wheel_ticks=0;
}

void GizWheel::clear_right_encoder() {
  right_wheel_ticks=0;
}

uint16_t GizWheel::get_left_wheel_ticks() {
  return(left_wheel_ticks);
}

uint16_t GizWheel::get_right_wheel_ticks() {
  return(right_wheel_ticks);
}

GizWheel::GizWheel(){
  if (!_initialized) {
    _initialized = true;
    attachInterrupt(digitalPinToInterrupt(LEFT_WHEEL_PIN), left_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_WHEEL_PIN), right_encoder_isr, CHANGE);
    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);
    pinMode(9, OUTPUT);
    digitalWrite(9, LOW);
  }
}

void GizWheel::update() {

// Serial.print("l:");
// Serial.println(left_wheel_ticks);
//
// Serial.print("r:");
// Serial.println(right_wheel_ticks); 


    //we sometime get bad data from wheel encoder
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
//    rwt_total+=rwt/8567.0*10006.0;

    double lDist_m=lwt*TICK_DISTANCE_M*DRIFT_CORRECTION_FACTOR;
    double rDist_m=rwt*TICK_DISTANCE_M;

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
    a += 2.0 * M_PI;
  }
  while(a > M_PI) {
    a -= 2.0 * M_PI;
  }
  return a;
}

void GizWheel::correct_position(Vec2d new_pos){
    x_pos_m=new_pos.x;
    y_pos_m=new_pos.y;
}

void GizWheel::correct_heading(double new_heading){
    heading_r=new_heading;
}


//double GizWheel::get_left_distance_m() {
//  const int ticks = left_wheel_ticks;
//  left_wheel_ticks -= ticks;
//  return ticks * WHEEL_SEPARATION_M;
//}
//
//double GizWheel::get_right_distance_m() {
//  const int ticks = right_wheel_ticks;
//  right_wheel_ticks -= ticks;
//  return ticks * WHEEL_SEPARATION_M;
//}
//
//double GizWheel::peek_left_distance_m() const {
//  return left_wheel_ticks * WHEEL_SEPARATION_M;
//}
//
//
//double GizWheel::peek_right_distance_m() const {
//  return right_wheel_ticks * WHEEL_SEPARATION_M;
//}
