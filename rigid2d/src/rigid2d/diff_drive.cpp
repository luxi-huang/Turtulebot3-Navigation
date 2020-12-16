#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include "cmath"
#include <cstdlib> //c standard library

namespace rigid2d{

DiffDrive::DiffDrive()
{
  Vector2D vec(0,0);
  Transform2D trans(vec, 0.0);
  pose_ = trans;

  whe_base = 4.0;
  whe_radius = 2.0;
}

DiffDrive::DiffDrive(Transform2D pose,double wheel_base, double wheel_radius)
{
  pose_ = pose;
  whe_base = wheel_base;
  whe_radius = wheel_radius;
}

WheelVelocities DiffDrive::twistToWheels(Twist2D twist)
{
  WheelVelocities vel;
  vel.ur = (2.0 * twist.vx + twist.theta_dot * whe_base) / (2 * whe_radius);
  vel.ul = (2.0 * twist.vx - twist.theta_dot * whe_base) / (2 * whe_radius);

  return vel;
}

Twist2D DiffDrive::wheelsToTwist(WheelVelocities vel){
  Twist2D t;

  // latest
  t.theta_dot = (vel.ur - vel.ul) * whe_radius / whe_base;
  t.vx = (vel.ur + vel.ul) * whe_radius / whe_base;
  t.vy = 0.0;
  return t;
}

void DiffDrive::updateOdometry(double left_radians, double right_radians)
{
  
  WheelVelocities wheel_v;
  wheel_v.ul = left_radians;
  wheel_v.ur = right_radians;
  Twist2D twist = wheelsToTwist(wheel_v);
  Transform2D transformation = integrateTwist(twist);
  pose_ *= transformation;
}

void DiffDrive::feedforward(Twist2D cmd) 
{
    Transform2D transformation;
    transformation = integrateTwist(cmd);
    pose_ *= transformation;
}

Transform2D DiffDrive::getpose()
{
  return pose_;
}

WheelVelocities DiffDrive::wheelVelocitie(double delta_radians_L, double delta_radians_R) const {
    WheelVelocities result;
    result.ul = delta_radians_L;
    result.ur = delta_radians_R;
    return result;
}

void DiffDrive::reset(Twist2D ps)
{
  Transform2D reset_pose(Vector2D(ps.vx, ps.vy), ps.theta_dot);
  pose_ = reset_pose;
}


// std::ostream & operator<<(std::ostream & os, const Pose & pose){
//   os << "Pose_theta " << pose.theta << " x " << pose.x << "y "<< pose.y;
//   return os;
// }

// std::istream & operator>>(std::istream & is, Pose & pose){
//   is >> pose.theta;
//   is >> pose.x;
//   is >> pose.y;
//   return is;
// }

// std::ostream & operator<<(std::ostream & os, const  WheelVelocities & wheel_v){
//   os << "u1 " << wheel_v.u1 << " u2 " << wheel_v.u2 << "u3 "<< wheel_v.u3 << "u4 "<< wheel_v.u4;
//   return os;
// }

// std::istream & operator>>(std::istream & is, WheelVelocities & wheel_v){
//   is >> wheel_v.u1;
//   is >> wheel_v.u2;
//   is >> wheel_v.u3;
//   is >> wheel_v.u4;
//   return is;
// }

}
