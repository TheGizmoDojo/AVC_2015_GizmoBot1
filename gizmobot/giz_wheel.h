#ifndef __GIZ_WHEEL
#define __GIZ_WHEEL
#include <stdint.h>
#include "Arduino.h"
#include <math.h>

class GizWheel {

public:
  double x_pos=0;
  double y_pos=0;
  double heading=1.5708;//90 degrees(ie: toward y)
  void update();
  void init();

private:
  static bool _initialized;
  double bound_angle(double a);

};


#endif  // __GIZ_WHEEL
