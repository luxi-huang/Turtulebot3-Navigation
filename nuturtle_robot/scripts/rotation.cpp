// odometer: update internal odometry state, publish odometry message ,and broadbast the  odom_frame_id;
//
// PUBLISHERS:nav_msgs/Odometry.h(odometry) publish the odometry message
//
// SUBSCRIBERS:sensor_msgs/JointState (joint_state): to get wheels velocity
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "rigid2d/set_pen.h"
#include <cmath>

using std::string;
using rigid2d::WheelVelocities;
using rigid2d::Pose;
using rigid2d::DiffDrive;
using rigid2d::Twist2D;
using rigid2d::PI;


ros::Publisher cmd_vel_publisher;

void publish_velocity(double rotation_velocity);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotation");
  ros::NodeHandle n;
  // odm_publisher = na.advertise<nav_msgs::Odometry>("Odometry", 1000);
  cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

  double frac_vel = 0.8;
  double maximum_rotational_velocity_robot = 2.84;
  n.getParam("frac_vel",frac_vel);
  n.getParam("maximum_rotational_velocity_robot", maximum_rotational_velocity_robot);

  double rotation_velocity = frac_vel*maximum_rotational_velocity_robot;
  double rotation_time = 2.0*PI/rotation_velocity;
  double pub_intervel = 0.02;
  double loop_number_double = rotation_time / pub_intervel;
  int loop_number_int = round(loop_number_double);

  ros::Time start_time,end_time;
  double duration;

  ros::Rate rate(1/pub_intervel);
  for (int j =0; j<20; j++){
    start_time = ros::Time::now();
    for(int i = 0; i<loop_number_int;i++){
      ros::spin();
      // cmd_vel_publisher.publish(rotation_velocity);
      publish_velocity(rotation_velocity);
      rate.sleep();
    }

    end_time = ros::Time::now();
    duration = (start_time - end_time).toSec();
    //pub zero velocity, stop 1/20 rotation time;
    publish_velocity(0);
    ros::Duration(duration/20.0);
  }
  return 0;
}

void publish_velocity(double rotation_velocity){
  geometry_msgs::Twist rotation_angle;
  rotation_angle.linear.x = 0;
  rotation_angle.linear.x = 0;
  rotation_angle.linear.x = 0;

  rotation_angle.angular.x = 0;
  rotation_angle.angular.y = 0;
  rotation_angle.angular.z = rotation_velocity;

  cmd_vel_publisher.publish(rotation_angle);
}
