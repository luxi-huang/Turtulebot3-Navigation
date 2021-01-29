/// \file
/// \brief Library for Waypoints class

#include "rigid2d/waypoints.hpp"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <iostream>
#include <cmath>
#include <cstdlib> //c standard library
#include <vector>
// #include "ros/ros.h"

namespace rigid2d{

void Waypoints::pipeline() {
  update_state ();
  Twist2D vel = nextWaypoint();
  update_pose(vel);
}  

Waypoints::Waypoints()
{
  points = {{0.0,0.0},{0.5,0.0},{0.5,1.0},{0.0,1.0}};
  static_vel.vx = 0.5;
  static_vel.vy = 0.0;
  static_vel.theta_dot = 0.5;
  current_goal = 1;
  frequency = 10;
  linearThreshold = 0.1;
  angularThreshold = 0.1;
}

Waypoints::Waypoints(std::vector<Vector2D> waypoint, Twist2D vel, DiffDrive my_diff, double hz, double l_thred, double a_thred)
{
  myDiffDrive = my_diff;
  points = waypoint;
  static_vel = vel;
  current_goal = 1;
  frequency = hz;
  linearThreshold = l_thred;
  angularThreshold = a_thred;
}

void Waypoints::update_state() 
{
  double current_pose_x, current_pose_y, current_pose_angle;
  Transform2D current_pose = myDiffDrive.getpose();
  current_pose.displacement(current_pose_x, current_pose_y, current_pose_angle);

  // if the current point is close to goal, we need to update goal;
  if (ifClose(current_pose_x, current_pose_y, points[current_goal].x, points[current_goal].y)) 
  {
    update_goal();
  } 

  double angle_diff = normalize_angle(std::atan2(points[current_goal].y - current_pose_y, points[current_goal].x - current_pose_x) - current_pose_angle);
  if (angle_diff < angularThreshold && - angularThreshold < angle_diff)
    {
      state_ = Trans;
    }
    else if (angle_diff > angularThreshold)
    {
      state_ = Rotate_left;
    }
    else
    {
      state_ = Rotate_right;
    }
  
  // if (ifClose(current_pose_x, current_pose_y, points[current_goal].x, points[current_goal].y)) 
  // {
  //   update_goal();
  // } 
}

bool Waypoints::ifClose(double x1, double y1, double x2, double y2) 
{
  return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2)) < linearThreshold;
}

void Waypoints::update_goal() 
{  
  current_goal ++;
  if (current_goal == points.size())
  {
    current_goal  = 0;
  }
}

Twist2D Waypoints::nextWaypoint() 
{
  Twist2D cmd; 
  if (state_ == Trans) {
    cmd = move_forward_cmd();
  } else if (state_ == Rotate_left) {
    cmd = rotate_left();
  } else if (state_ == Rotate_right) {
    cmd = rotate_right();
  }

  return cmd;
}

Twist2D Waypoints::move_forward_cmd() 
{
  Twist2D cmd;
  cmd.vx = static_vel.vx;
  cmd.vy = 0.0;
  cmd.theta_dot = 0.0;
  return cmd;
}

Twist2D Waypoints::rotate_left() 
{
  Twist2D cmd;
  cmd.vx = 0.0;
  cmd.vy = 0.0;
  cmd.theta_dot = static_vel.theta_dot;
  return cmd;
}

Twist2D Waypoints::rotate_right() 
{
  Twist2D cmd;
  cmd.vx = 0.0;
  cmd.vy = 0.0;
  cmd.theta_dot = - static_vel.theta_dot;
  return cmd;
}

void Waypoints::update_pose(Twist2D cmd) 
{
    // correct the twist according to frequency
  cmd.theta_dot = cmd.theta_dot * (1.0 / frequency);
  cmd.vx = cmd.vx * (1.0 / frequency);
  cmd.vy = cmd.vy * (1.0 / frequency);

  myDiffDrive.feedforward(cmd);
  double pose_x, pose_y, pose_theta;
  Transform2D pose = myDiffDrive.getpose();
  pose.displacement(pose_x, pose_y, pose_theta);
}

void  Waypoints::update_pose_close_loop(double x, double y,double theta)
{
  Transform2D pose = myDiffDrive.getpose();
  pose.displacement(x, y, theta);
}





void Waypoints::getPose(double &x, double &y, double &theta)
{
  Transform2D pose = myDiffDrive.getpose();
  pose.displacement(x, y, theta);
}

}
