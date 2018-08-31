#ifndef __GIZ_WHEEL
#define __GIZ_WHEEL

#include <stdint.h>


class GizWheel {
public:
  GizWheel();
  GizWheel(double wheel_separation_m);
  void update();
  double x_pos_m = 0;
  double y_pos_m = 0;
  double heading_r = 1.5708;//90 degrees(ie: toward y)

private:

  double get_left_distance_m();
  double get_right_distance_m();
  double peek_left_distance_m() const;
  double peek_right_distance_m() const;
  const double _wheel_separation_m;
  static bool _initialized;
  GizWheel(const GizWheel& rhs) = delete;
  GizWheel(GizWheel&& rhs) = delete;
  GizWheel& operator=(const GizWheel& rhs) = delete;
};


#endif  // __GIZ_WHEEL
