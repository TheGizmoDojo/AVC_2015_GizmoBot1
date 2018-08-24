#ifndef __GIZ_MOTOR
#define __GIZ_MOTOR
#include <Servo.h>

#define MOTOR_PIN 12
#define MOTOR_MID 90
#define MOTOR_SPEED 14 //20 is speedy
#define MOTOR_FORWARD MOTOR_MID-MOTOR_SPEED
#define MOTOR_REVERSE MOTOR_MID+MOTOR_SPEED



class GizMotor{

public:
  
public:
  void init();
  void forward();
  void stop();
  void reverse();
};



#endif 
