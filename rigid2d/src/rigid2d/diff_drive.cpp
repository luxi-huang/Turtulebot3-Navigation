#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include "cmath"
#include <cstdlib> //c standard library

using namespace rigid2d;

DiffDrive::DiffDrive(){
  x = 0.0;
  y = 0.0;
  theta = 0.0;
  whe_base = 4.0;
  whe_radius = 2.0;
  L = 5.0;
}

DiffDrive::DiffDrive(const Pose & p,double wheel_base, double wheel_radius){
  x = p.x;
  y = p.y;
  theta = p.theta;
  whe_base = wheel_base;
  whe_radius = wheel_radius;
}

WheelVelocities DiffDrive::twistToWheels(Twist2D t){
  WheelVelocities u;
  double D = whe_base/2;
  u.u1 = (1/whe_radius)*((-D)*t.theta_dot + t.vx);
  u.u2 = (1/whe_radius)*((D)*t.theta_dot + t.vx);
  u.u3 = u.u2;
  u.u4 = u.u1;
  return u;
}

Twist2D DiffDrive::wheelsToTwist(WheelVelocities vel){
  Twist2D t;
  double D = whe_base/2;
  // t.theta_dot = ((-1/(D+L))*vel.u1 + (1/(D+L))*vel.u2 + (1/(D+L))*vel.u3 - (1/(D+L))*vel.u4)*(whe_radius/4);
  // t.vx = vel.u1 + vel.u2 + vel.u3 + vel.u4;
  // t.vy = -vel.u1 + vel.u2 - vel.u3 + vel.u4;
  t.theta_dot = (vel.u1+vel.u2)*whe_radius/2;
  t.vx = whe_radius*(vel.u2-vel.u1)/(2*D);
  t.vy = 0;

  return t;
}

void DiffDrive::updateOdometry(double left_radians, double right_radians){
  double R_D,L_D,D_T = 0;
  R_D = whe_radius*right_radians;
  L_D = whe_radius*left_radians;
  theta += (R_D -L_D)/whe_base;
  D_T = (R_D + L_D)/2;
  x = -(D_T)*sin(theta);
  y = (D_T)*cos(theta);
}

void DiffDrive::feedforward(Twist2D cmd){
  double t = 1.0, new_theta;
  new_theta = cmd.theta_dot*t + theta;
  theta = normalize_angle(new_theta);
  double linear_velocity = sqrt(pow(cmd.vx,2)+pow(cmd.vy,2));
  x += linear_velocity*cos(theta)*t;
  y += linear_velocity*sin(theta)*t;
}
//
Pose DiffDrive::pose(){
  Pose p;
  p.x = x;
  p.y = y;
  p.theta = theta;
  return p;
}

WheelVelocities DiffDrive::wheelVelocitie(double delta_radians_L, double delta_radians_R) const {
  WheelVelocities u;
  double R_D = delta_radians_L*whe_radius;
  double L_D = delta_radians_R*whe_radius;
  u.u1 = R_D;
  u.u2 = L_D;
  u.u3 = u.u3;
  u.u4 = u.u1;
  return u;
}

std::ostream & operator<<(std::ostream & os, const Pose & pose){
  os << "Pose_theta " << pose.theta << " x " << pose.x << "y "<< pose.y;
  return os;
}

std::istream & operator>>(std::istream & is, Pose & pose){
  is >> pose.theta;
  is >> pose.x;
  is >> pose.y;
  return is;
}

std::ostream & operator<<(std::ostream & os, const  WheelVelocities & wheel_v){
  os << "u1 " << wheel_v.u1 << " u2 " << wheel_v.u2 << "u3 "<< wheel_v.u3 << "u4 "<< wheel_v.u4;
  return os;
}

std::istream & operator>>(std::istream & is, WheelVelocities & wheel_v){
  is >> wheel_v.u1;
  is >> wheel_v.u2;
  is >> wheel_v.u3;
  is >> wheel_v.u4;
  return is;
}