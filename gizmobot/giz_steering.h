#ifndef __GIZ_STEARING
#define __GIZ_STEARING
#include <Servo.h>

#define STEER_PIN 11
#define STEER_MID 82
#define STEER_AMOUNT 20
#define STEER_LEFT STEER_MID-STEER_AMOUNT
#define STEER_RIGHT STEER_MID+STEER_AMOUNT



class GizSteering{

public:
  
public:
  void init();
  void steer(double v);
};



#endif 
