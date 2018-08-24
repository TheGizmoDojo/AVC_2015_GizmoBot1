#include "giz_motor.h"

Servo motor;

void GizMotor::init(){
  motor.attach(MOTOR_PIN);
}

void GizMotor::forward(){
  motor.write(MOTOR_FORWARD);
}

void GizMotor::stop(){
  motor.write(MOTOR_MID);
}

void GizMotor::reverse(){
  motor.write(MOTOR_REVERSE);
}

