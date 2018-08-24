#include "giz_steering.h"

Servo steering;

void GizSteering::init(){
  steering.attach(STEER_PIN);
}
void GizSteering::steer(double v){
  steering.write(STEER_MID);
}


