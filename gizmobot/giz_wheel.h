#ifndef __GIZ_WHEEL
#define __GIZ_WHEEL
#include <stdint.h>
#include "Arduino.h"
#include <math.h>
#include "vec2d.h"

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
  void correct_position(Vec2d new_pos);
  void correct_heading(double new_heading);
  uint16_t get_left_wheel_ticks();
  uint16_t get_right_wheel_ticks();
  uint32_t lwt_total=0;
  uint32_t rwt_total=0;
    
private:

//  double get_left_distance_m();
//  double get_right_distance_m();
//  double peek_left_distance_m() const;
//  double peek_right_distance_m() const;
  static bool _initialized;
  double bound_angle_r(double a);

};


#endif  // __GIZ_WHEEL
