/*
* compass updates at a rate of about 100hz
*
*/
#ifndef __GIZ_GRYO_
#define __GIZ_GRYO_
#include "Arduino.h"
#define GRYO_SERIAL Serial2
#define GRYO_BAUD 57600

class GizGyro{

public:
  GizGyro();
  float heading_r;
  
public:
  void update();
};


#endif 
