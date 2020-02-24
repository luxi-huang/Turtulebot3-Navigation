#include "rigid2d/waypoints.hpp"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <iostream>
#include <cmath>
#include <cstdlib> //c standard library
#include <vector>
// #include "ros/ros.h"

namespace rigid2d{

Waypoints::Waypoints(){
  std::vector<Vector2D> points={{0.5,0.0},{1.0,0.5},{0.5,1.0}};
  std::vector<Velocity> vel = {{0,0}};
  std::vector<Twist2D> tw = {{0,0,0}};
  std::vector<Velocity> static_vel = {{0,0}};
  // goal = 1;
}

Waypoints::Waypoints(std::vector<Vector2D> p,Velocity v){
  points = p;
  static_vel[0] = v;
  vel[0] =v;
  goal = 1;
}

double Waypoints::left_distance(Pose pose){
  double left_d;
  left_d = sqrt(pow((pose.x - points[goal].x),2)+pow((pose.y-points[goal].y),2));

  // std::cout<<"goal "<< goal << "goal.x " << points[goal].x<< "goal.y "<< points[goal].y;
  return left_d;
}
//
double Waypoints::left_angle(Pose pose){
  Vector2D v1;
  double diff_angle;
  v1.x = points[goal].x - pose.x;
  v1.y = points[goal].y - pose.y;
  diff_angle = v1.angle(v1);
  diff_angle -= pose.theta;
  diff_angle = normalize_angle(diff_angle);
  return diff_angle;
}

void Waypoints::convert_velocity_to_twist(){
  tw.clear();
  tw[0].theta_dot = vel[0].angular;
  tw[0].vx = vel[0].linear * cos(vel[0].angular);
  // tw[0].vy = vel[0].linear * sin(vel[0].angular);
  tw[0].vy = 0;
}

void Waypoints::change_goal(){
  if (goal < points.size()-1){
    goal = goal+1;
  } else{
    goal =0;
  }
}

// // get another array of velocity and chagne goal to next goal;
Twist2D Waypoints::nextWaypoint(double rest_distance,double rest_angle,double threshold_linear,double threshold_angular){
  Twist2D twist;
  // std::cout << "goal"<<goal;
  if ((rest_angle>0 && static_vel[0].angular<0)||(rest_angle<0 && static_vel[0].angular>0))
  {
    static_vel[0].angular = - static_vel[0].angular;
  }

  if(std::abs(rest_angle) >= threshold_angular)
  {
    vel[0].linear = 0;
    vel[0].angular = static_vel[0].angular;
    // std::cout<< "11111111111";
  }else
  {
    // ROS_INFO_STREAM("Rico test: rest_angle - threshold_angular, should be < 0: "<<rest_angle<<"threshold_angular: "<<threshold_angular);
    // ROS_INFO(rest_angle);
    // std::cout<<"rest angle"<<rest_angle<<std::endl;
    if (std::abs(rest_distance) >=threshold_linear)
    {
      vel[0].linear = static_vel[0].linear;
      vel[0].angular = 0;
      // std::cout << "222222222";
    } else
    {
      vel[0].linear = 0;
      vel[0].angular = 0;
      change_goal();
      // std::cout<< "33333333333";
    }
  }
  convert_velocity_to_twist();
  twist.vx = tw[0].vx;
  // std::cout<< "twist.vx!!!"<< twist.vx;
  // std::cout<< "static_vel.x!!!"<< static_vel[0].linear;
  twist.vy = tw[0].vy;
  twist.theta_dot = tw[0].theta_dot;
  return twist;
}

  int Waypoints::print_goal(){
    return goal;
  }


}
