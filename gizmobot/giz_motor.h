#ifndef __GIZ_MOTOR
#define __GIZ_MOTOR
#include <Servo.h>

#define MOTOR_PIN 12
#define MOTOR_MID 90
#define MOTOR_SPEED 6 //6 is slow, 20 is speedy
#define MOTOR_FORWARD MOTOR_MID-MOTOR_SPEED
#define MOTOR_REVERSE MOTOR_MID+MOTOR_SPEED



class GizMotor{

public:
  GizMotor();
  void forward();
  void stop();
  void reverse();

private:
  GizMotor(const GizMotor&) = delete;
  GizMotor(GizMotor&&) = delete;
  GizMotor& operator=(const GizMotor&) = delete;
};



#endif 
