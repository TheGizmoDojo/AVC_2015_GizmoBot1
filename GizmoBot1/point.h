#ifndef POINT_H
#define POINT_H

#include <iosfwd>


struct Point {
  float x_m;
  float y_m;
  Point(float x, float y) : x_m(x), y_m(y) {}
  float headingTo_r(const Point& other) const;
  float distanceTo_m(const Point& other) const;
  float relativeHeadingTo_r(float heading_r, const Point& other) const;
  friend std::ostream& operator<<(std::ostream& out, const Point& rhs);
};

// Given how far the left and right wheels have turned, calculate the change in
// heading, and the direction and distance to the new position. Note that these
// values are in radians to avoid converting before using the trigonometric
// functions. Note that the change in heading and direction to the new position
// are different; for example, if we make a constant 90 degree turn, our
// heading changes by 90 degrees, but the direction to the new position is 45
// degrees.
void computeHeadingDistance(float leftDistance_m, float rightDistance_m, float* heading_r, float* direction_r, float* distance_m);

#endif
