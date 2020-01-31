#include "rigid2d/rigid2d.hpp"
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
  v.x = v.x*T11 + v.y*T12 + T13;
  v.y = v.x*T21 + v.y*T22 + T23;
  return v;
}
//

Transform2D::Transform2D(){
  T11 = 1;
  T12 = 0;// Vector2D m;
  T13 = 0;
  T21 = 0;
  T22 = 1;
  T23 = 0;
  degree = 0;
}

Transform2D::Transform2D(const Vector2D & trans){
  T11 = 1;
  T12 = 0;// Vector2D m;
  T13 = trans.x;
  T21 = 0;
  T22 = 1;
  T23 = trans.y;
  degree = 0;
}

Transform2D::Transform2D(double radians){
  T11 = cos(radians);
  T12 = -sin(radians);
  T13 = 0;
  T21 = sin(radians);
  T22 = cos(radians);
  T23 = 0;
  degree = rad2deg(radians);

}
Transform2D::Transform2D(const Vector2D & trans, double radians){
  T11 = cos(radians);
  T12 = -sin(radians);
  T13 = trans.x;
  T21 = sin(radians);
  T22 = cos(radians);
  T23 = trans.y;
  degree = rad2deg(radians);
}

Transform2D Transform2D::inv() const{
  Transform2D T;
  T.T11 = T11;
  T.T12 = T21;
  T.T21 = T12;
  T.T22 = T22;
  T.T13 = -T11*T13 -T21*T23;
  T.T23 = -T12*T13 -T22*T23;
  T.degree = -degree;
  return T;
}

Transform2D& Transform2D::operator*=(const Transform2D & rhs){
  Transform2D Q;
  Q.T11 = T11*rhs.T11 + T12*rhs.T21;
  Q.T12 = T11*rhs.T12 + T12*rhs.T22;
  Q.T21 = T21*rhs.T11 + T22*rhs.T21;
  Q.T22 = T21*rhs.T12 + T22*rhs.T22;
  Q.T13 = T11*rhs.T13 + T12*rhs.T23 + T13;
  Q.T23 = T21*rhs.T13 + T22*rhs.T23 + T23;

  if (Q.T11 < 0){
  Q.degree = rad2deg(std::acos(Q.T11));
  }else{
  Q.degree = rad2deg(std::atan(-Q.T12/Q.T11));
  }
  // Q.degree = rad2deg(asin(Q.T21));

  *this = Q;
  return *this;
}

std::ostream & rigid2d::operator<<(std::ostream & os, const Transform2D & tf){
  os << "degree" << tf.degree << " dx " << tf.T13 << "dy "<< tf.T23;
  return os;
}


std::istream & rigid2d::operator>>(std::istream & is, Transform2D & tf){
  Vector2D v;
  double degree, radians;
  std::cout <<"x";
  is >> v.x;
  std::cout << "y";
  is >> v.y;
  std::cout << "degree";
  is >> degree;
  radians = deg2rad(degree);

  Transform2D tf_new(v, radians);
  tf = tf_new;
  return is;
}

Transform2D rigid2d::operator*(Transform2D lhs, const Transform2D & rhs){
  lhs*=rhs;
  // A = lhs;
  return lhs;
}

std::ostream & rigid2d::operator<<(std::ostream & os, const Twist2D & twist){
  os << "degree_dt" << twist.theta_dot << " vx " << twist.vx << "vy "<< twist.vy;
  return os;
}

std::istream & operator>>(std::istream & is, Twist2D & twist){
  is >> twist.theta_dot;
  is >> twist.vx;
  is >> twist.vy;
  return is;
}

Twist2D Transform2D::operator()(Twist2D tw) const{
  Twist2D twist;
  twist.theta_dot = tw.theta_dot;
  twist.vx = tw.theta_dot*T23 +T11*tw.vx + T12*tw.vy;
  twist.vy = -tw.theta_dot*T13+T21*tw.vx + T22*tw.vy;
  return twist;
}


Vector2D rigid2d::intergrateTwist(const Twist2D & twi){
  Vector2D v;
  v.x = twi.vx*1;
  v.y = twi.vy*1;
  return v;
}
