//DIGITAL PINS USABLE FOR INTERRUPTS: 2, 3, 18, 19, 20, 21
#define WHEEL_LEFT_PIN 2
#define WHEEL_RIGHT_PIN 3
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>
using namespace std;

#include "GizmoBot1/point.h"

static void test();
static void testPoint();
static void testComputeArcTurnAngle();
static void testComputeArcTurnDistance();
static void testComputeHeadingDistance();
static float toRadians(float degrees);


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


float toRadians(const float degrees) {
  return degrees / 180 * M_PI;
}


void testPoint() {
  const Point start(0, 0);

  float heading_r;
  float relativeHeading_r;

  heading_r = start.headingTo_r(Point(0, 1));
  assert((heading_r >= 0 && heading_r < toRadians(1)) || (heading_r > toRadians(359) && heading_r <= toRadians(360)));

  heading_r = start.headingTo_r(Point(1, 3));
  assert(heading_r > 0 && heading_r < toRadians(45));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(1, 4));
  assert(relativeHeading_r < 0 && relativeHeading_r > toRadians(-30));

  heading_r = start.headingTo_r(Point(0, 1));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(1, 1));
  assertClose(relativeHeading_r, toRadians(45));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(1, 0));
  assertClose(relativeHeading_r, toRadians(90));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(1, -1));
  assertClose(relativeHeading_r, toRadians(135));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(0.0001, -1));
  assertClose(relativeHeading_r, toRadians(180));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(-0.0001, -1));
  assertClose(relativeHeading_r, toRadians(-180));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(-1, -1));
  assertClose(relativeHeading_r, toRadians(-135));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(-1, 0));
  assertClose(relativeHeading_r, toRadians(-90));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(-1, 1));
  assertClose(relativeHeading_r, toRadians(-45));

  heading_r = start.headingTo_r(Point(1, -1));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(1, 1));
  assertClose(relativeHeading_r, toRadians(-90));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(1, 0));
  assertClose(relativeHeading_r, toRadians(-45));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(1, -1));
  assertClose(relativeHeading_r, toRadians(0));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(0, -1));
  assertClose(relativeHeading_r, toRadians(45));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(-1, -1));
  assertClose(relativeHeading_r, toRadians(90));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(-1, 0));
  assertClose(relativeHeading_r, toRadians(135));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(-1, 0.9999));
  assertClose(relativeHeading_r, toRadians(180));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(-1, 1.0001));
  assertClose(relativeHeading_r, toRadians(-180));
  relativeHeading_r = start.relativeHeadingTo_r(heading_r, Point(0, 1));
  assertClose(relativeHeading_r, toRadians(-135));
}


void testComputeArcTurnAngle() {
  void _computeArcTurn(float leftDistance_m, float rightDistance_m, float wheelSeparation_m, float* const angle_r);

  float angle1_r, angle2_r;

  // One wheel turns
  _computeArcTurn(M_PI / 4, 0.0, 1.0, &angle1_r);
  assertClose(angle1_r, toRadians(45));
  _computeArcTurn(0.0, M_PI / 4, 1.0, &angle2_r);
  assertClose(angle1_r, -angle2_r);

  _computeArcTurn(M_PI / 2, 0.0, 1.0, &angle1_r);
  assertClose(angle1_r, toRadians(90));
  _computeArcTurn(0.0, M_PI / 2, 1.0, &angle2_r);
  assertClose(angle1_r, -angle2_r);

  _computeArcTurn(M_PI, 0.0, 1.0, &angle1_r);
  assertClose(angle1_r, toRadians(180));
  _computeArcTurn(0.0, M_PI, 1.0, &angle2_r);
  assertClose(angle1_r, -angle2_r);

  _computeArcTurn(2 * M_PI, 0.0, 1.0, &angle1_r);
  assertClose(angle1_r, toRadians(360));  // Or 0
  _computeArcTurn(0.0, 2 * M_PI, 1.0, &angle2_r);
  assertClose(angle1_r, -angle2_r);

  // Wheels turn same
  _computeArcTurn(1.0, 1.0, 1.0, &angle1_r);
  assertClose(angle1_r, 0);

  _computeArcTurn(4.0, 4.0, 1.0, &angle1_r);
  assertClose(angle1_r, 0);

  _computeArcTurn(0.0, 0.0, 1.0, &angle1_r);
  assertClose(angle1_r, 0);

  // One wheel turns slightly more
  _computeArcTurn(10.0, 10.001, 100.0, &angle1_r);
  assert(angle1_r < 0);
  assertClose(angle1_r, 0);

  _computeArcTurn(10.001, 10.0, 100.0, &angle1_r);
  assert(angle1_r > 0);
  assertClose(angle1_r, 0);

  // Other random tests
  _computeArcTurn(1.0, 2.0, 1.0, &angle1_r);
  assert(toRadians(-60) < angle1_r && angle1_r < toRadians(-45));
}


void testComputeArcTurnDistance() {
  void _computeArcTurn(float leftDistance_m, float rightDistance_m, float wheelSeparation_m, float* const angle_r);
  void _computeArcTurn(float leftDistance_m, float rightDistance_m, float wheelSeparation_m, float* const angle_r, float* const innerRadius_m);

  float angle1_r, angle2_r;
  float innerRadius_m;

  // These should use the same calculation for angle
  _computeArcTurn(1.0, 2.0, 0.5, &angle1_r);
  _computeArcTurn(1.0, 2.0, 0.5, &angle2_r, &innerRadius_m);
  assertClose(angle1_r, angle2_r);

  // One wheel turns
  _computeArcTurn(1.0, 0.0, 1.0, &angle1_r, &innerRadius_m);
  assertClose(innerRadius_m, 0.0);
  _computeArcTurn(0.0, 1.0, 1.0, &angle1_r, &innerRadius_m);
  assertClose(innerRadius_m, 0.0);

  _computeArcTurn(2.0, 0.0, 1.0, &angle1_r, &innerRadius_m);
  assertClose(innerRadius_m, 0.0);
  _computeArcTurn(0.0, 2.0, 1.0, &angle1_r, &innerRadius_m);
  assertClose(innerRadius_m, 0.0);

  // Wheels turn same
  _computeArcTurn(1.0, 1.0, 1.0, &angle1_r, &innerRadius_m);
  assert(innerRadius_m > 100000);  // Essentially infinite

  _computeArcTurn(4.0, 4.0, 1.0, &angle1_r, &innerRadius_m);
  assert(innerRadius_m > 100000);  // Essentially infinite

  // Uneven turn
  _computeArcTurn(2.0, 4.0, 1.0, &angle1_r, &innerRadius_m);
  assertClose(innerRadius_m, 1.0);
  _computeArcTurn(4.0, 2.0, 1.0, &angle1_r, &innerRadius_m);
  assertClose(innerRadius_m, 1.0);

  // One wheel turns slightly more
  _computeArcTurn(10.0, 10.001, 100.0, &angle1_r, &innerRadius_m);
  assert(angle1_r < 0);
  assert(innerRadius_m > 10000);

  _computeArcTurn(10.001, 10.0, 100.0, &angle1_r, &innerRadius_m);
  assert(angle1_r > 0);
  assert(innerRadius_m > 10000);
}


void testComputeHeadingDistance() {
  void _computeHeadingDistance(const float leftDistance_m, const float rightDistance_m, const float wheelSeparation_m, float* const heading_r, float* direction_r, float* const distance_m);

  float heading_r;
  float direction_r;
  float distance_m;

  // *** Right turns ***
  // One wheel turns
  _computeHeadingDistance(60. / 180 * M_PI, 0.0, 1.0, &heading_r, &direction_r, &distance_m);
  assertClose(heading_r, toRadians(60));
  assertClose(direction_r, toRadians(30));
  assertClose(distance_m, 0.5);

  _computeHeadingDistance(M_PI / 2, 0.0, 1.0, &heading_r, &direction_r, &distance_m);
  assertClose(heading_r, toRadians(90));
  assertClose(direction_r, toRadians(45));
  assertClose(distance_m, 0.5 * sqrt(2));

  _computeHeadingDistance(M_PI, 0.0, 1.0, &heading_r, &direction_r, &distance_m);
  assertClose(heading_r, toRadians(180));
  assertClose(direction_r, toRadians(90));
  assertClose(distance_m, 1);

  // Uneven turn
  _computeHeadingDistance(2.5, 2.0, 1.0, &heading_r, &direction_r, &distance_m);
  assert(0 < heading_r && heading_r < 45);
  assert(heading_r > direction_r);
  assert(2.0 < distance_m && distance_m < 2.5);

  // One wheel turns slightly more
  _computeHeadingDistance(10.0, 10.001, 100.0, &heading_r, &direction_r, &distance_m);
  assert(heading_r < 0);

  // *** Left turns ***
  // One wheel turns
  _computeHeadingDistance(0.0, 60.0 / 180 * M_PI, 1.0, &heading_r, &direction_r, &distance_m);
  assertClose(heading_r, toRadians(-60));
  assertClose(direction_r, toRadians(-30));
  assertClose(distance_m, 0.5);

  _computeHeadingDistance(0.0, M_PI / 2, 1.0, &heading_r, &direction_r, &distance_m);
  assertClose(heading_r, toRadians(-90));
  assertClose(direction_r, toRadians(-45));
  assertClose(distance_m, 0.5 * sqrt(2));

  _computeHeadingDistance(0.0, M_PI, 1.0, &heading_r, &direction_r, &distance_m);
  assertClose(heading_r, toRadians(-180));
  assertClose(direction_r, toRadians(270));  // Or -90
  assertClose(distance_m, 1);

  // Uneven turn
  _computeHeadingDistance(2.0, 2.5, 1.0, &heading_r, &direction_r, &distance_m);
  assert(-45 < heading_r && heading_r < 0);
  assert(heading_r < direction_r);
  assert(2.0 < distance_m && distance_m < 2.5);

  // One wheel turns slightly more
  _computeHeadingDistance(10.001, 10.0, 100.0, &heading_r, &direction_r, &distance_m);
  assert(heading_r > 0);

  // *** Straight ***
  _computeHeadingDistance(1.0, 1.0, 1.0, &heading_r, &direction_r, &distance_m);
  assertClose(heading_r, toRadians(0));
  assertClose(direction_r, toRadians(0));
  assertClose(distance_m, 1.0);

  _computeHeadingDistance(4.0, 4.0, 1.0, &heading_r, &direction_r, &distance_m);
  assertClose(heading_r, toRadians(0));
  assertClose(direction_r, toRadians(0));
  assertClose(distance_m, 4.0);
}
