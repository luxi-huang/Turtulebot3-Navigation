#include "rigid2d.hpp"
#include <iostream>
#include "cmath"
#include <cstdlib> //c standard library


using namespace rigid2d;

constexpr double PI = 3.14159265358979323846;

constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12){
  if (abs(d1-d2)<epsilon){
    return true;
  }else{
    return false;
  }
}

constexpr double deg2rad(double deg)
{
  return deg * rigid2d::PI /180;
}

constexpr double rad2deg(double rad)
{
  return rad * 180 /rigid2d::PI;
}

std::ostream & operator<<(std::ostream & os, const Vector2D & v){
  os << "< " << v.x << " , " << v.y << " >";
  return os;
}

std::istream & operator>>(std::istream & is, Vector2D & v){
  is >> v.x;
  is >> v.y;
  return is;
}
