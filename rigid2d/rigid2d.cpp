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

Vector2D Transform2D::operator()(Vector2D v) const{
  v.x = v.x + xt;
  v.y = v.y + yt;
  v.x = v.x * xr1 + v.y * xr2;
  v.y = v.x * yr1 + v.y * yr2;
  return v;
}
//

Transform2D::Transform2D(){
  xt = 0;
  yt = 0;
}

Transform2D::Transform2D(const Vector2D & trans){
  xt = trans.x;
  yt = trans.y;
  xr1 = 1;
  xr2 = 0;
  yr1 = 0;
  yr2 = 1;
}

Transform2D::Transform2D(double radians){
  xt = 0;
  yt = 0;
  xr1 = cos(radians);
  xr2 = -sin(radians);
  yr1 = sin(radians);
  yr2 = cos(radians);
}
Transform2D::Transform2D(const Vector2D & trans, double radians){
  xt = trans.x;
  yt = trans.y;
  xr1 = cos(radians);
  xr2 = -sin(radians);
  yr1 = sin(radians);
  yr2 = cos(radians);
}

// Transform2D inv() const{
//   xt = 0;
// }
