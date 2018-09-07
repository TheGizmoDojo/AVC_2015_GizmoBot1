/*
* compass updates at a rate of about 100hz
*
*/
#ifndef __GIZ_COMPASS__
#define __GIZ_COMPASS__
#include "Arduino.h"

class GizCompass{

public:
  GizCompass();
  double heading_r;
  void update();
  void init();
};


#endif 
