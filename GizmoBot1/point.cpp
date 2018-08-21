#include "point.h"
#include <algorithm>
#include <cmath>
#include <ostream>
using std::min;
using std::ostream;
using std::sqrt;


float Point::headingTo_d(const Point& other) const {
  float angle = -((atan2f(other.y_m - y_m, other.x_m - x_m) / M_PI * 180) - 90);
  if (angle < 0) {
    angle += 360;
  }
  return angle;
}


float Point::distanceTo_m(const Point& other) const {
  const float diffX_m = other.x_m - x_m;
  const float diffY_m = other.y_m - y_m;
  return sqrt(diffX_m * diffX_m + diffY_m * diffY_m);
}


float Point::relativeHeadingTo_d(float heading_d, const Point& other) const {
  const float otherHeading_d = headingTo_d(other);
  double difference = otherHeading_d - heading_d;
  while (difference < -180) {
    difference += 360;
  }
  while (difference > 180) {
    difference -= 360;
  }
  return difference;
}


ostream& operator<<(ostream& out, const Point& rhs) {
  out << rhs.x_m << " " << rhs.y_m;
  return out;
}


// This is purposely left out of the header file. Users should use the function
// that assumes the wheel_separation. This isn't defined as static for testing.
void _computeArcTurn(const float leftDistance_m, const float rightDistance_m, const float wheelSeparation_m, float* const angle_d, float* const innerRadius_m) {
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
  *angle_d = (leftDistance_m - rightDistance_m) / (2 * M_PI * wheelSeparation_m) * 360;
  if (*angle_d == 0) {
    *innerRadius_m = 1000000;
  } else {
    *innerRadius_m = min(leftDistance_m, rightDistance_m) / (fabs(*angle_d / 360) * (2 * M_PI));
  }
}


// This is purposely left out of the header file. Users should use the function
// that assumes the wheel_separation. This isn't defined as static for testing.
void _computeArcTurn(const float leftDistance_m, const float rightDistance_m, const float wheelSeparation_m, float* const angle_d) {
  *angle_d = (leftDistance_m - rightDistance_m) / (2 * M_PI * wheelSeparation_m) * 360;
}


static const float WHEEL_SEPARATION_M = 0.175;


void computeArcTurn(const float leftDistance_m, const float rightDistance_m, float* const angle_d) {
  return _computeArcTurn(leftDistance_m, WHEEL_SEPARATION_M, rightDistance_m, angle_d);
}


void computeArcTurn(const float leftDistance_m, const float rightDistance_m, float* const angle_d, float* const innerRadius_m) {
  return _computeArcTurn(leftDistance_m, WHEEL_SEPARATION_M, rightDistance_m, angle_d, innerRadius_m);
}


// This is purposely left out of the header file. Users should use the function
// that assumes the wheel_separation. This isn't defined as static for testing.
void _computeHeadingDistance(const float leftDistance_m, const float rightDistance_m, const float wheelSeparation_m, float* const heading_d, float* direction_d, float* const distance_m) {
  if (leftDistance_m == rightDistance_m) {
    *heading_d = 0;
    *distance_m = leftDistance_m;
    *direction_d = 0;
    return;
  }
  float innerRadius_m;
  _computeArcTurn(leftDistance_m, rightDistance_m, wheelSeparation_m, heading_d, &innerRadius_m);
  const float midRadius_m = innerRadius_m + (0.5 * wheelSeparation_m);
  // Let the middle of the wheels be the origin.
  // Rotate the vector from pivot point to middle of wheels by heading_d.
  // This rotation matrix is in the clockwise direction.
  // |  cos sin |   | +/-midRadius_m |
  // | -sin cos | X |        0       |
  // That gives us (x, y) but that's from the point of view of the pivot point.
  const float xVector_m = leftDistance_m > rightDistance_m ? -midRadius_m : midRadius_m;
  const float rotation_d = leftDistance_m > rightDistance_m ? -*heading_d : *heading_d;
  const float xPointFromPivot_m = cos(rotation_d * M_PI / 180) * xVector_m;
  const float yPointFromPivot_m = sin(rotation_d * M_PI / 180) * xVector_m;
  // But we want it from the point of view of the car's midpoint
  float xPoint_m = midRadius_m + xPointFromPivot_m;
  const float yPoint_m = yPointFromPivot_m;
  *distance_m = sqrt((xPoint_m * xPoint_m) + (yPoint_m * yPoint_m));
  *direction_d = 90 - atan2f(yPoint_m, xPoint_m) * 180 / M_PI;
  if (*heading_d < 0) {
    *direction_d = -*direction_d;
  }
}


void computeHeadingDistance(const float leftDistance_m, const float rightDistance_m, float* const heading_d, float* const direction_d, float* const distance_m) {
  return _computeHeadingDistance(leftDistance_m, rightDistance_m, WHEEL_SEPARATION_M, heading_d, direction_d, distance_m);
}
