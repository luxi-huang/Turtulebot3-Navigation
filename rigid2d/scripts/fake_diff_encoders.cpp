// fake_diff_encoders:create_fake_diff_encoders;
//
// PUBLISHERS:sensor_msgs/JointState (joint_state): publish wheels's joint_state
//
// SUBSCRIBERS:geometry_msgs/Twist (cmd_vel);
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "ros/ros.h"
#include <sstream>
#include "geometry_msgs/Twist.h"
#include <cstdlib>
#include <sensor_msgs/JointState.h>
#include <string>
#include <nav_msgs/Odometry.h>

using namespace std; // use for count
using namespace rigid2d;

ros::Publisher joint_state_publisher;
ros::Subscriber vel_subscriber ;
Twist2D Twist_value;
// std::vector<std::string> joint_name_vector;

void velCallback(const geometry_msgs::Twist::ConstPtr & Twist);
void publish_joint_state(WheelVelocities v);

string left_wheel_joint;
string right_wheel_joint;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_diff_encoders");

  ros::NodeHandle nh;


  // string left_wheel_joint;
  // string right_wheel_joint;
  nh.getParam("odometer/left_wheel_joint", left_wheel_joint);
  nh.getParam("odometer/right_wheel_joint",right_wheel_joint);

  double wheel_base;
  double wheel_radius;
  nh.getParam("wheel_base",wheel_base);
  nh.getParam("wheel_radius",wheel_radius);
  // ROS_INFO ("wheel_radius,%f",wheel_radius);
  // wheel_base = 4.0;
  // wheel_radius =2.0;
  DiffDrive diff1;


  Pose pp;
  pp.x = 0;
  pp.y = 0;
  pp.theta = 0;

  diff1 = DiffDrive(pp,wheel_base,wheel_radius);

  vel_subscriber = nh.subscribe("turtle1/cmd_vel",10,velCallback);
  joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

  while(ros::ok()){
    // ROS_INFO("%f",Twist_value.vx);
    WheelVelocities wheel_v;
    wheel_v = diff1.twistToWheels(Twist_value);
    // diff1.feedforward(Twist_value);
    publish_joint_state(wheel_v);
    // ROS_INFO ("wheel_radius,%f",wheel_radius);
    // ROS_INFO("%s",left_wheel_joint.c_str());
    // ROS_INFO("fake_x %f",Twist_value.theta_dot);

    ros::spinOnce();
  }






  return 0;
}

void velCallback(const geometry_msgs::Twist::ConstPtr & Twist){
  Twist_value.vx = Twist->linear.x;
  Twist_value.vy = Twist->linear.y;
  Twist_value.theta_dot=Twist->angular.z ;
}


void publish_joint_state(WheelVelocities v){
  std::vector<std::string> joint_name_vector;
  sensor_msgs::JointState j_s;
  joint_name_vector.push_back("odometer/left_wheel_joint");
  joint_name_vector.push_back("odometer/right_wheel_joint");

  j_s.name.resize(joint_name_vector.size());
  j_s.position.resize(joint_name_vector.size());
  j_s.velocity.resize(joint_name_vector.size());

  j_s.name[0] = joint_name_vector[0];
  j_s.name[1] = joint_name_vector[1];

  j_s.velocity[0] = v.u1;
  j_s.velocity[1] = v.u2;
  joint_state_publisher.publish(j_s);




}
