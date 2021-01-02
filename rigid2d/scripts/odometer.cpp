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
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
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
  double last_L_pose, last_R_pose;
  double frequency = 60;
  bool init_flag;

public: 
  Odometer(ros::NodeHandle &nh) 
  {
    odm_publisher = nh.advertise<nav_msgs::Odometry>("odom", 10);
    joint_state_subscriber = nh.subscribe("joint_state", 1000, &Odometer::poseCallback, this);

    ros::param::get("~odom_frame_id",odom_frame_id);
    ros::param::get("~body_frame_id",body_frame_id);
    nh.getParam("left_wheel_joint",left_wheel_joint);
    nh.getParam("right_wheel_joint",right_wheel_joint);
    
    nh.getParam("base_link",base_link);
    nh.getParam("wheel_base",wheel_base);
    nh.getParam("wheel_radius",wheel_radius);

    // initial robot;
    rigid2d::Transform2D initPose;
    myRobot = rigid2d::DiffDrive(initPose, wheel_base, wheel_radius); 

    last_time = ros::Time::now();;
    init_flag = false;
    ros::Rate rate(frequency);
    while(ros::ok())
    {
      rigid2d::Transform2D current_pose = myRobot.getpose();
      pub_odm(current_pose);
      send_TF(current_pose);
      rate.sleep();
      ros::spinOnce();
      
    }
  }

  void poseCallback(const sensor_msgs::JointState::ConstPtr & mesg)
  {
    if (!init_flag) 
    {
      last_L_pose =  mesg->position[0];
      last_R_pose = mesg->position[1];
      init_flag = true;
    }
    
    wheel1.name = mesg->name[0];
    wheel2.name = mesg->name[1];
    wheel1.position = mesg->position[0];
    wheel2.position = mesg->position[1];
  }

  void send_TF(rigid2d::Transform2D pose)
  {
    double x, y, theta;
    pose.displacement(x, y, theta);

    static tf2_ros::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = body_frame_id;
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();

    odom_broadcaster.sendTransform(odom_trans);
  }

  void pub_odm (rigid2d::Transform2D current_pose)
  {
    double left_dist = wheel1.position - last_L_pose;
    left_dist = rigid2d::normalize_angle(left_dist);
    double right_dist =  wheel2.position - last_R_pose;
    right_dist = rigid2d::normalize_angle(right_dist);

    // get position 
    myRobot.updateOdometry(left_dist,right_dist);
    
    double pose_x, pose_y, pose_theta;
    current_pose.displacement(pose_x, pose_y, pose_theta);
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_theta);
    
    // get wheel velocity
    current_time = ros::Time::now();
    double vel_l = (left_dist) / (current_time - last_time).toSec();
    double vel_r = (right_dist) / (current_time - last_time).toSec();
    rigid2d::WheelVelocities wheel_vel;
    wheel_vel.ul = vel_l;
    wheel_vel.ur = vel_r;
    rigid2d::Twist2D twist = myRobot.wheelsToTwist(wheel_vel);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_id;
    //set the position
    odom.pose.pose.position.x = pose_x;
    odom.pose.pose.position.y = pose_y;
    odom.pose.pose.position.z = 0.0;

    // convert orientation to quaternion
    tf2::Quaternion q_rot;
    double r = 0, p = 0, y = pose_theta;
    q_rot.setRPY(r, p, y);
    q_rot.normalize();
    tf2::convert(q_rot, odom.pose.pose.orientation);

      //set the velocity
    odom.child_frame_id = body_frame_id;
    odom.twist.twist.linear.x = twist.vx;
    odom.twist.twist.linear.y = twist.vy;
    odom.twist.twist.angular.z = twist.theta_dot;
    odm_publisher.publish(odom);

    last_L_pose = wheel1.position;
    last_R_pose = wheel2.position;
    last_time = current_time;
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