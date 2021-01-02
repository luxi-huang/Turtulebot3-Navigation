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
  // std:: cout << "whe_radius: " << whe_radius <<"\n";
  // std:: cout << "whe_base: " << whe_base <<"\n";
  vel.ur = (2.0 * twist.vx + twist.theta_dot * whe_base) / (2 * whe_radius);
  vel.ul = (2.0 * twist.vx - twist.theta_dot * whe_base) / (2 * whe_radius);

  return vel;
}

Twist2D DiffDrive::wheelsToTwist(WheelVelocities vel){
  Twist2D t;

  // latest
  t.vx = (vel.ur + vel.ul) * whe_radius / 2.0;
  t.theta_dot = (vel.ur - vel.ul) * whe_radius / whe_base;
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
    double x, y, theta; 
    transformation.displacement(x, y, theta);
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

}
