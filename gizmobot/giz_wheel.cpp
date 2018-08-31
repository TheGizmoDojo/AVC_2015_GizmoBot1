#include "giz_wheel.h"

const int LEFT_WHEEL_PIN = 2;
const int RIGHT_WHEEL_PIN = 3;
const double WHEEL_SEPARATION_M = .175;
const double WHEEL_DIAMETER = 0.079;
const double WHEEL_CIRCUMFERENCE =WHEEL_DIAMETER * PI;
const double TICK_DISTANCE = WHEEL_CIRCUMFERENCE/14;

bool GizWheel::_initialized = false;
static uint16_t left_wheel_ticks = 0;
static uint16_t right_wheel_ticks = 0;


static void left_encoder_isr() {
  left_wheel_ticks++;
}

static void right_encoder_isr() {
  right_wheel_ticks++;
}

void GizWheel::init(){
    Serial.begin(115200);

  if (!_initialized) {
    _initialized = true;
    attachInterrupt(digitalPinToInterrupt(LEFT_WHEEL_PIN), left_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_WHEEL_PIN), right_encoder_isr, CHANGE);
    _initialized = true;
  }
}


double lwt_total=0;
double rwt_total=0;

void GizWheel::update(){

 Serial.print("l:");
 Serial.println(left_wheel_ticks);
    /// Serial.println(lDist);

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

    double lDist=lwt*TICK_DISTANCE;
    double rDist=rwt*TICK_DISTANCE;


	Serial.print("left Total:");
	Serial.println(lwt_total);
    /// Serial.println(lDist);

	Serial.print("Right Total:");
	Serial.println(rwt_total); 

 //Serial.print(",");
/// Serial.println(rDist);

    //reset ticks
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

double GizWheel::bound_angle(double a) {
	while(a <= -1*PI) {
		a += 2*PI;
	}
	while(a > PI) {
		a -= 2*PI;
	}
	return a;
}
