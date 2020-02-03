#include "rigid2d/waypoints.hpp"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <iostream>
#include "cmath"
#include <cstdlib> //c standard library
#include <vector>

using namespace rigid2d;
Waypoints::Waypoints(Vector2D G1,Vector2D G2,Vector2D G3,Vector2D G4){
  //not sure if it is correct;
  point1[0]=G1;
  point2[0]=G2;
  point3[0]=G3;
  point4[0]=G4;
  // G4 = goal;
}

double Waypoints::left_distance(Pose pose){
  double left_d;
  left_d = sqrt(pow((pose.x - goal.x),2)+pow((pose.y-goal.y),2));
  return left_d;
}

double Waypoints::left_angle(Pose pose){
  Vector2D v1;
  double diff_angle;
  v1.x = goal.x - pose.x;
  v1.y = goal.y -pose.y;
  diff_angle = v1.angle(v1);
  diff_angle -= pose.theta;
  return diff_angle;
}

void Waypoints::velocity_array(double rest_distance,double rest_angle,double steps){
  // int steps =1 +1 ;
  vel.clear();
  double linear_v, angular_v;
  linear_v = rest_distance/steps;
  angular_v = rest_angle/steps;

  for (int i =0;i<steps;i++){
    vel[i].linear = linear_v;
    vel[i].linear = angular_v;
  }
}

void Waypoints::convert_velocity_to_twist(){
  tw.clear();
  int size;
  size = vel.size();
  for (int i =0; i < size; i++){
    tw[2*i].theta_dot = vel[i].angular;
    tw[2*i].vx = vel[i].linear * cos(vel[i].angular);
    tw[2*i].vy = vel[i].linear * sin(vel[i].angular);
  }
}

void Waypoints::update_current_pose(DiffDrive &a){
 int size;
 size = tw.size();
 for (int i =0; i< size; i++){
   Twist2D t;
   a.feedforward(tw[i]);
 }
}


void Waypoints::change_goal(){
  if (goal.x == point1[0].x && goal.y == point1[0].y){
    goal = point2[1];
  } else if (goal.x == point2[0].x && goal.y == point2[0].y) {
    goal = point3[1];
  }else if (goal.x == point3[0].x && goal.y == point3[0].y) {
    goal = point4[1];
  }else if (goal.x == point4[0].x && goal.y == point4[0].y) {
    goal = point1[1];
  }
}

// get another array of velocity and chagne goal to next goal;
void Waypoints::nextWaypoint(double rest_distance,double rest_angle,double threshold_linear){
  if (rest_distance < threshold_linear){
    change_goal();
  }
}
