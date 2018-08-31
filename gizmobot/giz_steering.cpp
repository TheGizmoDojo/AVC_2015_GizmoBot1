#include "giz_steering.h"

Servo steering;

GizSteering::GizSteering(){
  steering.attach(STEER_PIN);
}
void GizSteering::steer(double v){
  steering.write(STEER_MID);
}


