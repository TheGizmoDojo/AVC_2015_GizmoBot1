#include "vec2d.h"

Vec2d::Vec2d(){
 	x = 0.0;
    y = 0.0;
}

Vec2d::Vec2d(double sx, double sy){
    x = sx;
    y = sy;
}

Vec2d Vec2d::operator+(Vec2d &v){
    return Vec2d(x+v.x, y+v.y);
}

Vec2d Vec2d::operator-(Vec2d &v){
    return Vec2d(x-v.x, y-v.y);
}

Vec2d Vec2d::operator*(Vec2d &v){
    return Vec2d(x*v.x, y*v.y);
}

Vec2d Vec2d::operator/(Vec2d &v){
    return Vec2d(x/v.x, y/v.y);
}

double Vec2d::dot(Vec2d &b){
    return ((x * b.x) + (y * b.y));
}

double Vec2d::cross(Vec2d &b){
    return ((x * b.y) - (y * b.x));
}

double Vec2d::directionTo_r(Vec2d &b){
    return atan2(b.x-x,b.y-y);
}

double Vec2d::directionFrom_r(Vec2d &b){
    return atan2(y-b.y,x-b.x);
}

double Vec2d::distanceTo(Vec2d &b){
    return sqrt(
        (b.x-x) * (b.x-x) +
        (b.y-y) * (b.y-y)
    );
}
