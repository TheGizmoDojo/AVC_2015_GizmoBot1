//DIGITAL PINS USABLE FOR INTERRUPTS: 2, 3, 18, 19, 20, 21
#define WHEEL_LEFT_PIN 2
#define WHEEL_RIGHT_PIN 3
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>
using namespace std;

#include "paths.h"
#include "point.h"

void test();
void testPoint();
void testComputeArcTurnAngle();
void testComputeArcTurnDistance();
void testComputeHeadingDistance();


int main() {
  test();
}

#define assertClose(v1, v2) assert(v1 - 0.01 < v2 && v2 < v1 + 0.01)
void test() {
  testPoint();
  testComputeArcTurnAngle();
  testComputeArcTurnDistance();
  testComputeHeadingDistance();
}


void testPoint() {
  const Point start(0, 0);

  float heading;
  float relativeHeading;

  heading = start.headingTo_d(Point(0, 1));
  assert((heading >= 0 && heading < 1) || (heading > 359 && heading <= 360));

  heading = start.headingTo_d(Point(1, 3));
  assert(heading > 0 && heading < 45);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(1, 4));
  assert(relativeHeading < 0 && relativeHeading > -30);

  heading = start.headingTo_d(Point(0, 1));
  relativeHeading = start.relativeHeadingTo_d(heading, Point(1, 1));
  assertClose(relativeHeading, 45);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(1, 0));
  assertClose(relativeHeading, 90);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(1, -1));
  assertClose(relativeHeading, 135);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(0.0001, -1));
  assertClose(relativeHeading, 180);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(-0.0001, -1));
  assertClose(relativeHeading, -180);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(-1, -1));
  assertClose(relativeHeading, -135);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(-1, 0));
  assertClose(relativeHeading, -90);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(-1, 1));
  assertClose(relativeHeading, -45);

  heading = start.headingTo_d(Point(1, -1));
  relativeHeading = start.relativeHeadingTo_d(heading, Point(1, 1));
  assertClose(relativeHeading, -90);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(1, 0));
  assertClose(relativeHeading, -45);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(1, -1));
  assertClose(relativeHeading, 0); relativeHeading = start.relativeHeadingTo_d(heading, Point(0, -1));
  assertClose(relativeHeading, 45);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(-1, -1));
  assertClose(relativeHeading, 90);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(-1, 0));
  assertClose(relativeHeading, 135);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(-1, 0.9999));
  assertClose(relativeHeading, 180);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(-1, 1.0001));
  assertClose(relativeHeading, -180);
  relativeHeading = start.relativeHeadingTo_d(heading, Point(0, 1));
  assertClose(relativeHeading, -135);
}


void testComputeArcTurnAngle() {
  void _computeArcTurn(float leftDistance_m, float rightDistance_m, float wheelSeparation_m, float* const angle_d);

  float angle1_d, angle2_d;

  // One wheel turns
  _computeArcTurn(M_PI / 4, 0.0, 1.0, &angle1_d);
  assertClose(angle1_d, 45);
  _computeArcTurn(0.0, M_PI / 4, 1.0, &angle2_d);
  assertClose(angle1_d, -angle2_d);

  _computeArcTurn(M_PI / 2, 0.0, 1.0, &angle1_d);
  assertClose(angle1_d, 90);
  _computeArcTurn(0.0, M_PI / 2, 1.0, &angle2_d);
  assertClose(angle1_d, -angle2_d);

  _computeArcTurn(M_PI, 0.0, 1.0, &angle1_d);
  assertClose(angle1_d, 180);
  _computeArcTurn(0.0, M_PI, 1.0, &angle2_d);
  assertClose(angle1_d, -angle2_d);

  _computeArcTurn(2 * M_PI, 0.0, 1.0, &angle1_d);
  assertClose(angle1_d, 360);
  _computeArcTurn(0.0, 2 * M_PI, 1.0, &angle2_d);
  assertClose(angle1_d, -angle2_d);

  // Wheels turn same
  _computeArcTurn(1.0, 1.0, 1.0, &angle1_d);
  assertClose(angle1_d, 0);

  _computeArcTurn(4.0, 4.0, 1.0, &angle1_d);
  assertClose(angle1_d, 0);

  _computeArcTurn(0.0, 0.0, 1.0, &angle1_d);
  assertClose(angle1_d, 0);

  // One wheel turns slightly more
  _computeArcTurn(10.0, 10.001, 100.0, &angle1_d);
  assert(angle1_d < 0);
  assertClose(angle1_d, 0);

  _computeArcTurn(10.001, 10.0, 100.0, &angle1_d);
  assert(angle1_d > 0);
  assertClose(angle1_d, 0);

  // Other random tests
  _computeArcTurn(1.0, 2.0, 1.0, &angle1_d);
  assert(-60 < angle1_d && angle1_d < -45);
}


void testComputeArcTurnDistance() {
  void _computeArcTurn(float leftDistance_m, float rightDistance_m, float wheelSeparation_m, float* const angle_d);
  void _computeArcTurn(float leftDistance_m, float rightDistance_m, float wheelSeparation_m, float* const angle_d, float* const innerRadius_m);

  float angle1_d, angle2_d;
  float innerRadius_m;

  // These should use the same calculation for angle
  _computeArcTurn(1.0, 2.0, 0.5, &angle1_d);
  _computeArcTurn(1.0, 2.0, 0.5, &angle2_d, &innerRadius_m);
  assert(angle1_d == angle2_d);

  // One wheel turns
  _computeArcTurn(1.0, 0.0, 1.0, &angle1_d, &innerRadius_m);
  assertClose(innerRadius_m, 0.0);
  _computeArcTurn(0.0, 1.0, 1.0, &angle1_d, &innerRadius_m);
  assertClose(innerRadius_m, 0.0);

  _computeArcTurn(2.0, 0.0, 1.0, &angle1_d, &innerRadius_m);
  assertClose(innerRadius_m, 0.0);
  _computeArcTurn(0.0, 2.0, 1.0, &angle1_d, &innerRadius_m);
  assertClose(innerRadius_m, 0.0);

  // Wheels turn same
  _computeArcTurn(1.0, 1.0, 1.0, &angle1_d, &innerRadius_m);
  assert(innerRadius_m > 100000);  // Essentially infinite

  _computeArcTurn(4.0, 4.0, 1.0, &angle1_d, &innerRadius_m);
  assert(innerRadius_m > 100000);  // Essentially infinite

  // Uneven turn
  _computeArcTurn(2.0, 4.0, 1.0, &angle1_d, &innerRadius_m);
  assertClose(innerRadius_m, 1.0);
  _computeArcTurn(4.0, 2.0, 1.0, &angle1_d, &innerRadius_m);
  assertClose(innerRadius_m, 1.0);

  // One wheel turns slightly more
  _computeArcTurn(10.0, 10.001, 100.0, &angle1_d, &innerRadius_m);
  assert(angle1_d < 0);
  assert(innerRadius_m > 10000);

  _computeArcTurn(10.001, 10.0, 100.0, &angle1_d, &innerRadius_m);
  assert(angle1_d > 0);
  assert(innerRadius_m > 10000);
}


void testComputeHeadingDistance() {
  void _computeHeadingDistance(const float leftDistance_m, const float rightDistance_m, const float wheelSeparation_m, float* const heading_d, float* direction_d, float* const distance_m);

  float heading_d;
  float direction_d;
  float distance_m;

  // *** Right turns ***
  // One wheel turns
  _computeHeadingDistance(60. / 180 * M_PI, 0.0, 1.0, &heading_d, &direction_d, &distance_m);
  assertClose(heading_d, 60);
  assertClose(direction_d, 30);
  assertClose(distance_m, 0.5);

  _computeHeadingDistance(M_PI / 2, 0.0, 1.0, &heading_d, &direction_d, &distance_m);
  assertClose(heading_d, 90);
  assertClose(direction_d, 45);
  assertClose(distance_m, 0.5 * sqrt(2));

  _computeHeadingDistance(M_PI, 0.0, 1.0, &heading_d, &direction_d, &distance_m);
  assertClose(heading_d, 180);
  assertClose(direction_d, 90);
  assertClose(distance_m, 1);

  // Uneven turn
  _computeHeadingDistance(2.5, 2.0, 1.0, &heading_d, &direction_d, &distance_m);
  assert(0 < heading_d && heading_d < 45);
  assert(heading_d > direction_d);
  assert(2.0 < distance_m && distance_m < 2.5);

  // One wheel turns slightly more
  _computeHeadingDistance(10.0, 10.001, 100.0, &heading_d, &direction_d, &distance_m);
  assert(heading_d < 0);

  // *** Left turns ***
  // One wheel turns
  _computeHeadingDistance(0.0, 60.0 / 180 * M_PI, 1.0, &heading_d, &direction_d, &distance_m);
  assertClose(heading_d, -60);
  assertClose(direction_d, -30);
  assertClose(distance_m, 0.5);

  _computeHeadingDistance(0.0, M_PI / 2, 1.0, &heading_d, &direction_d, &distance_m);
  assertClose(heading_d, -90);
  assertClose(direction_d, -45);
  assertClose(distance_m, 0.5 * sqrt(2));

  _computeHeadingDistance(0.0, M_PI, 1.0, &heading_d, &direction_d, &distance_m);
  assertClose(heading_d, -180);
  assertClose(direction_d, -90);
  assertClose(distance_m, 1);

  // Uneven turn
  _computeHeadingDistance(2.0, 2.5, 1.0, &heading_d, &direction_d, &distance_m);
  assert(-45 < heading_d && heading_d < 0);
  assert(heading_d < direction_d);
  assert(2.0 < distance_m && distance_m < 2.5);

  // One wheel turns slightly more
  _computeHeadingDistance(10.001, 10.0, 100.0, &heading_d, &direction_d, &distance_m);
  assert(heading_d > 0);

  // *** Straight ***
  _computeHeadingDistance(1.0, 1.0, 1.0, &heading_d, &direction_d, &distance_m);
  assertClose(heading_d, 0);
  assertClose(direction_d, 0);
  assertClose(distance_m, 1.0);

  _computeHeadingDistance(4.0, 4.0, 1.0, &heading_d, &direction_d, &distance_m);
  assertClose(heading_d, 0);
  assertClose(direction_d, 0);
  assertClose(distance_m, 4.0);
}
