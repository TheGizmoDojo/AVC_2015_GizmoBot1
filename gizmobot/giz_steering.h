#ifndef __GIZ_STEARING
#define __GIZ_STEARING
#include <Servo.h>

const int STEER_PIN = 11;
const int STEER_MID = 31;
// From observation, min = 21, max = 49
const int STEER_AMOUNT = 8;
const int STEER_LEFT = STEER_MID - STEER_AMOUNT;
const int STEER_RIGHT = STEER_MID + STEER_AMOUNT;
const int MAX_STEER_SWING = 10;
const int MAX_STEER_LEFT = STEER_MID - MAX_STEER_SWING;
const int MAX_STEER_RIGHT = STEER_MID + MAX_STEER_SWING;

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
