#include "point.h"
#include <algorithm>
#include <cmath>
#include <ostream>
using std::min;
using std::ostream;
using std::sqrt;


float Point::headingTo_r(const Point& other) const {
  float angle = -((atan2f(other.y_m - y_m, other.x_m - x_m)) - M_PI / 2);
  if (angle < 0) {
    angle += 2 * M_PI;
  }
  return angle;
}


float Point::distanceTo_m(const Point& other) const {
  const float diffX_m = other.x_m - x_m;
  const float diffY_m = other.y_m - y_m;
  return sqrt(diffX_m * diffX_m + diffY_m * diffY_m);
}


float Point::relativeHeadingTo_r(float heading_r, const Point& other) const {
  const float otherHeading_r = headingTo_r(other);
  double difference = otherHeading_r - heading_r;
  while (difference < -M_PI) {
    difference += 2 * M_PI;
  }
  while (difference > M_PI) {
    difference -= 2 * M_PI;
  }
  return difference;
}


ostream& operator<<(ostream& out, const Point& rhs) {
  out << rhs.x_m << " " << rhs.y_m;
  return out;
}


// This is purposely left out of the header file. Users should use the function
// that assumes the wheel_separation. This isn't defined as static so that it
// can be used for testing.
void _computeArcTurn(const float leftDistance_m, const float rightDistance_m, const float wheelSeparation_m, float* const angle_r, float* const innerRadius_m) {
  // Let x = distance from pivot point to inner wheel, unknown
  // Let y = distance between wheels, known
  // Let a1 = length of closer arc, inner wheel
  // Let a2 = length of further arc, outer wheel
  // Let m = angle
  // Then:
  // (m / 360) * 2 * pi * x = a1
  // x = a1 / ((m / 360) * 2 * pi)
  // And:
  // (m / 360) * 2 * pi * (x + y) = a2
  // x + y = a2 / ((m / 360) * 2 * pi)
  // x = (a2 / ((m / 360) * 2 * pi)) - y
  // Solving both sides yields:
  // m = (a2 - a1) / (2 * pi * y) * 360

  // TODO: Optimize this; factor out some common multiplications and divisions
  if (leftDistance_m == rightDistance_m) {
    *innerRadius_m = 1000000;
    *angle_r = 0;
  } else {
    const float fraction = (leftDistance_m - rightDistance_m) / (2 * M_PI * wheelSeparation_m);
    *angle_r = fraction * 2 * M_PI;
    *innerRadius_m = min(leftDistance_m, rightDistance_m) / (fabs(fraction) * 2 * M_PI);
  }
}


// This is purposely left out of the header file. Users should use the function
// that assumes the wheel_separation. This isn't defined as static so that it
// can be used for testing.
void _computeArcTurn(const float leftDistance_m, const float rightDistance_m, const float wheelSeparation_m, float* const angle_r) {
  *angle_r = (leftDistance_m - rightDistance_m) / (2 * M_PI * wheelSeparation_m) * 2 * M_PI;
}


static const float WHEEL_SEPARATION_M = 0.175;


// This is purposely left out of the header file. Users should use the function
// that assumes the wheel_separation. This isn't defined as static for testing.
void _computeHeadingDistance(const float leftDistance_m, const float rightDistance_m, const float wheelSeparation_m, float* const heading_r, float* direction_r, float* const distance_m) {
  if (leftDistance_m == rightDistance_m) {
    *heading_r = 0;
    *distance_m = leftDistance_m;
    *direction_r = 0;
    return;
  }
  float innerRadius_m;
  _computeArcTurn(leftDistance_m, rightDistance_m, wheelSeparation_m, heading_r, &innerRadius_m);
  const float midRadius_m = innerRadius_m + (0.5 * wheelSeparation_m);
  // Let the middle of the wheels be the origin.
  // Rotate the vector from pivot point to middle of wheels by heading_d.
  // This rotation matrix is in the clockwise direction.
  // |  cos sin |   | +/-midRadius_m |
  // | -sin cos | X |        0       |
  // That gives us (x, y) but that's from the point of view of the pivot point.
  const bool rightTurn = leftDistance_m > rightDistance_m;
  const float xVector_m = rightTurn ? -midRadius_m : midRadius_m;
  const float xPointFromPivot_m = cos(*heading_r) * xVector_m;
  const float yPointFromPivot_m = -sin(*heading_r) * xVector_m;
  // But we want it from the point of view of the car's midpoint
  const float xPoint_m = (rightTurn ? midRadius_m : -midRadius_m) + xPointFromPivot_m;
  const float yPoint_m = yPointFromPivot_m;
  *distance_m = sqrt((xPoint_m * xPoint_m) + (yPoint_m * yPoint_m));
  // atan2f returns the angle from (1, 0) in the counter-clockwise direction. We
  // want the angle from (0, 1) in the clockwise direction.
  *direction_r = M_PI / 2 - atan2f(yPoint_m, xPoint_m);
}


void computeHeadingDistance(const float leftDistance_m, const float rightDistance_m, float* const heading_r, float* const direction_r, float* const distance_m) {
  return _computeHeadingDistance(leftDistance_m, rightDistance_m, WHEEL_SEPARATION_M, heading_r, direction_r, distance_m);
}
