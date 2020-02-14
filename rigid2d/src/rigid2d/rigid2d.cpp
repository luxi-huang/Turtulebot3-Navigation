#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include "cmath"
#include <cstdlib> //c standard library

namespace rigid2d{

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

std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
  os << "degree" << tf.degree << " dx " << tf.T13 << "dy "<< tf.T23;
  return os;
}


std::istream & operator>>(std::istream & is, Transform2D & tf){
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

Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
  lhs*=rhs;
  // A = lhs;
  return lhs;
}

std::ostream & operator<<(std::ostream & os, const Twist2D & twist){
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


//Vector multiplication, scaler on left side;

Vector2D operator*(double s, const Vector2D v)
{
  Vector2D a;
  a.x = v.x*s;
  a.y = v.y*s;
  return a;
}

//Vector multiplication, scaler on left side;
Vector2D operator*=(double a, Vector2D &v)
{
  v.x = v.x*a;
  v.y = v.y*a;
  return v;
}

// std::ostream & operator<<(std::ostream & os, const Twist2D & twist){
//   os << "degree_dt" << twist.theta_dot << " vx " << twist.vx << "vy "<< twist.vy;
//   return os;
// }

Transform2D displacement(const Transform2D & T){
  Transform2D t;
  t = T;
  return t;
}


// Transform2D rigid2d::integrateTwist(const Twist2D & V, const Transform2D & T){
//
// }
}
