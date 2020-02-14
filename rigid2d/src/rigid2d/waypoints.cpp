#include "rigid2d/waypoints.hpp"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <iostream>
#include "cmath"
#include <cstdlib> //c standard library
#include <vector>

namespace rigid2d{
Waypoints::Waypoints(std::vector<Vector2D> p,Velocity v){
  points = p;
  static_vel[0] = v;
  vel[0] =v;
}



double Waypoints::left_distance(Pose pose){
  double left_d;
  left_d = sqrt(pow((pose.x - points[goal].x),2)+pow((pose.y-points[goal].y),2));
  return left_d;
}
//
double Waypoints::left_angle(Pose pose){
  Vector2D v1;
  double diff_angle;
  v1.x = points[goal].x - points[goal].x;
  v1.y = points[goal].y - points[goal].y;
  diff_angle = v1.angle(v1);
  diff_angle -= pose.theta;
  return diff_angle;
}
//
// void velocity_array(double rest_distance,double rest_angle,double steps){
//   // int steps =1 +1 ;
//   vel.clear();
//   double linear_v, angular_v;
//   linear_v = rest_distance/steps;
//   angular_v = rest_angle/steps;
//
//   for (int i =0;i<steps;i++){
//     vel[i].linear = linear_v;
//     vel[i].linear = angular_v;
//   }
// }
//
void Waypoints::convert_velocity_to_twist(){
  tw.clear();
  tw[0].theta_dot = vel[0].angular;
  tw[0].vx = vel[0].linear * cos(vel[0].angular);
  tw[0].vy = vel[0].linear * sin(vel[0].angular);
}
//
void Waypoints::update_current_pose(DiffDrive &a, double ttime){
 int size;
 size = tw.size();
 for (int i =0; i< size; i++){
   Twist2D t;
   a.feedforward(tw[i],ttime);
 }
}
//
//
void Waypoints::reset_velocity(){
  vel[0] = static_vel[0];
}

void Waypoints::change_goal(){
  if (goal < points.size()-1){
    goal = goal+1;
  } else{
    goal =0;
  }
  reset_velocity();
}

// // get another array of velocity and chagne goal to next goal;
void Waypoints::nextWaypoint(double rest_distance,double rest_angle,double threshold_linear){
  reset_velocity();
  if (rest_distance < threshold_linear){
    //change goal
    change_goal();
  }else if (rest_distance < vel[0].linear){
    //change linear speed;
    vel[0].linear = rest_distance;
    if(rest_angle < vel[0].angular){
    //change angular speed;
      vel[0].angular = rest_angle;
    }
  }
}

}
