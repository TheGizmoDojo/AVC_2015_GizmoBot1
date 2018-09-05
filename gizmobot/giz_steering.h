#ifndef __GIZ_STEARING
#define __GIZ_STEARING
#include <Servo.h>

#define STEER_PIN 11
//#define STEER_MID 82
#define STEER_MID 87
#define STEER_AMOUNT 20
#define STEER_LEFT STEER_MID-STEER_AMOUNT
#define STEER_RIGHT STEER_MID+STEER_AMOUNT
#define MAX_STEER_SWING 15
#define MAX_STEER_LEFT STEER_MID-MAX_STEER_SWING
#define MAX_STEER_RIGHT STEER_MID+MAX_STEER_SWING

class GizSteering{
  
public:
  void init();
  GizSteering();
  void steer(int v);
private:
  GizSteering(const GizSteering&) = delete;
  GizSteering(GizSteering&&) = delete;
  GizSteering& operator=(const GizSteering&) = delete;
};



#endif 
