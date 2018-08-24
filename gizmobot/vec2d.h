#ifndef __VEC2D_
#define __VEC2D_
#include <math.h>
class Vec2d {
public:
    double x;
    double y;

    Vec2d();
    Vec2d(double, double);

    Vec2d operator+(Vec2d&);
    Vec2d operator-(Vec2d&);
    Vec2d operator*(Vec2d&);
    Vec2d operator/(Vec2d&);

    double dot(Vec2d&);
    double cross(Vec2d&);
    double directionTo(Vec2d&);
    double directionFrom(Vec2d&);
    double distance(Vec2d&);

};
#endif 

