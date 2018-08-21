#ifndef POINT_H
#define POINT_H

#include <iosfwd>


struct Point {
  float x_m;
  float y_m;
  Point(float x, float y) : x_m(x), y_m(y) {}
  float headingTo_d(const Point& other) const;
  float distanceTo_m(const Point& other) const;
  float relativeHeadingTo_d(float heading_d, const Point& other) const;
  friend std::ostream& operator<<(std::ostream& out, const Point& rhs);
};


// GIven how far the left and right wheels have turned, calculate the rotation
// and radius to the inner wheel around some pivot point
void computeArcTurn(float leftDistance_m, float rightDistance_m, float* angle_d);
void computeArcTurn(float leftDistance_m, float rightDistance_m, float* angle_d, float* innerRadius_m);

// Given how far the left and right wheels have turned, calculate the change in
// heading, and the direction and distance to the new position. Note that the
// change in heading and direction to the new position are different; for
// example, if we make a constant 90 degree turn, our heading changes by 90
// degrees, but the direction to the new position is 45 degrees.
void computeDistanceDelta(float leftDistance_m, float rightDistance_m, float* heading_d, float* direction_d, float* distance_m);

#endif
