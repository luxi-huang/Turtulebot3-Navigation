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

using std::string;
using rigid2d::WheelVelocities;
using rigid2d::Pose;
using rigid2d::DiffDrive;
using rigid2d::Twist2D;


ros::Publisher odm_publisher;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotation");
  ros::NodeHandle na;
  odm_publisher = na.advertise<nav_msgs::Odometry>("Odometry", 1000);

  while(ros::ok()){
    ros::spinOnce();
  }
  return 0;
}
