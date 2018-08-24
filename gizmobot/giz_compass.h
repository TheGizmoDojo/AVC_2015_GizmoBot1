/*
* compass updates at a rate of about 100hz
*
*/
#ifndef __GIZ_COMPASS_
#define __GIZ_COMPASS_
#include "Arduino.h"
#define COMPASS_SERIAL Serial2
#define COMPASS_BAUD 57600

class GizCompass{

public:
  float heading;
  
public:
  void init();
  void update();
};


#endif 
