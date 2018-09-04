#ifndef __GIZ_WHEEL
#define __GIZ_WHEEL
#include <stdint.h>
#include "Arduino.h"
#include <math.h>

class GizWheel {

public:
  double x_pos_m=0;
  double y_pos_m=0;
  double heading_r=1.5708;//90 degrees(ie: toward y)
  void update();
  void init();
  GizWheel();
  void clear_left_encoder();
  void clear_right_encoder();

private:

  double get_left_distance_m();
  double get_right_distance_m();
  double peek_left_distance_m() const;
  double peek_right_distance_m() const;
  static bool _initialized;
  double bound_angle_r(double a);

};


#endif  // __GIZ_WHEEL
