/// \file
/// \brief: update internal odometry state, publish odometry message ,and broadbast the  odom_frame_id;
///
/// PUBLISHERS:nav_msgs/Odometry.h(odometry) publish the odometry message
///
/// SUBSCRIBERS:sensor_msgs/JointState (joint_state): to get wheels velocity

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "ros/ros.h"
#include <sstream>
#include "geometry_msgs/Twist.h"
#include <cstdlib>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// #include "geometry_msgs/Pose.h"
#include "rigid2d/set_pen.h"

using std::string;

struct MyJointState {
  string name;
  double velocity;
  double position;
  MyJointState(
    string n = "",
    double v = 0.0,
    double p = 0.0): name(n), velocity(v), position(p)
    {}
};

class Odometer
{

private: 
  ros::Publisher odm_publisher;
  ros::Subscriber joint_state_subscriber;
  string odom_frame_id;
  string body_frame_id;
  string left_wheel_joint;
  string right_wheel_joint;
  string base_link;
  double wheel_base;
  double wheel_radius;
  rigid2d::DiffDrive myRobot;
  MyJointState wheel1,wheel2;

  ros::Time current_time;
  ros::Time last_time;
  double duration;
  
public: 
  Odometer(ros::NodeHandle &nh) 
  {
    odm_publisher = nh.advertise<nav_msgs::Odometry>("odom", 1000);
    joint_state_subscriber = nh.subscribe("joint_state", 1000, &Odometer::poseCallback, this);

    nh.getParam("~odom_frame_id",odom_frame_id);
    nh.getParam("~body_frame_id",body_frame_id);
    nh.getParam("~left_wheel_joint",left_wheel_joint);
    nh.getParam("~right_wheel_joint",right_wheel_joint);
    
    nh.getParam("base_link",base_link);
    nh.getParam("wheel_base",wheel_base);
    nh.getParam("wheel_radius",wheel_radius);

    // initial robot;
    rigid2d::Transform2D initPose;
    myRobot = rigid2d::DiffDrive(initPose, wheel_base, wheel_radius); 

    current_time = ros::Time::now();
    last_time = current_time;
    // ros::Duration(0.5).sleep();

    while(ros::ok())
    {
      current_time = ros::Time::now();
      duration = (current_time - last_time).toSec();
      last_time = current_time;
      rigid2d::Twist2D tw;
      rigid2d::WheelVelocities wheel_v = wheel_vel();

      tw = myRobot.wheelsToTwist(wheel_v);
      tw.theta_dot *= duration;
      tw.theta_dot = rigid2d::normalize_angle(theta_dot);
      tw.vx *= duration;
      tw.vy *= duration;
      myRobot.feedforward(tw);
 
      rigid2d::Transform2D pose = myRobot.getpose();
      // P = myRobot.pose();
      send_TF(pose);
      pub_odm (pose,tw);
      ros::spinOnce();
    }
  }

  void poseCallback(const sensor_msgs::JointState::ConstPtr & mesg)
  {
    wheel1.name = mesg->name[0];
    wheel2.name = mesg->name[1];
    wheel1.position = mesg->position[0];
    wheel2.position = mesg->position[1];
    wheel2.velocity = mesg->velocity[1];
    wheel1.velocity = mesg->velocity[0];
  }

  rigid2d::WheelVelocities wheel_vel()
  {
    rigid2d:: WheelVelocities wheelVel;
    wheelVel.ul = wheel1.velocity;
    wheelVel.ur = wheel2.velocity;
    return wheelVel;
  }

  void send_TF(rigid2d::Transform2D pose)
  {
    double x, y, theta;
    pose.displacement(x, y, theta);

    static tf2_ros::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_link;
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = theta;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();

    odom_broadcaster.sendTransform(odom_trans);
  }

  void pub_odm (rigid2d::Transform2D pose,rigid2d::Twist2D tw)
  {
    double x, y, theta;
    pose.displacement(x, y, theta);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_id;
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

      //set the velocity
    odom.child_frame_id = base_link;
    odom.twist.twist.linear.x = tw.vx;
    odom.twist.twist.linear.y = tw.vy;
    odom.twist.twist.angular.z = tw.theta_dot;
    odm_publisher.publish(odom);
  }


};

int main(int argc, char **argv)
{
    // init the node
    ros::init(argc, argv, "odometer");
    ros::NodeHandle nh;
    Odometer odomClass = Odometer(nh);

    ros::spin();
    return 0;
}