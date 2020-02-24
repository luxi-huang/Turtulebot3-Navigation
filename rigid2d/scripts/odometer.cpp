// odometer: update internal odometry state, publish odometry message ,and broadbast the  odom_frame_id;
//
// PUBLISHERS:nav_msgs/Odometry.h(odometry) publish the odometry message
//
// SUBSCRIBERS:sensor_msgs/JointState (joint_state): to get wheels velocity
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
 // use for count
using rigid2d::WheelVelocities;
using rigid2d::Pose;
using rigid2d::DiffDrive;
using rigid2d::Twist2D;


ros::Publisher odm_publisher;
ros::Subscriber joint_state_subscriber;
ros::ServiceServer service;
string odom_frame_id;
string base_link;

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

MyJointState wheel1,wheel2;

void poseCallback(const sensor_msgs::JointState::ConstPtr & mesg);
WheelVelocities wheel_vel();
void send_TF(Pose P,ros::Time current_time);
void pub_odm (Pose P,Twist2D tw,ros::Time current_time);
int set_pose_client(Pose pp);
DiffDrive diffa;

bool set_pose_callback(rigid2d::set_pen::Request  &req,
                rigid2d::set_pen::Response &resp)
{
  Pose p;
  ros::Time current_time;
  Twist2D tw;
  tw.vx = 0;
  tw.vy = 0;
  tw.theta_dot = 0;

  current_time = ros::Time::now();
  p.x = req.robot_pose.x;
  p.y = req.robot_pose.y;
  p.theta = req.robot_pose.theta;


  ros::NodeHandle nh;
  double wheel_base;
  double wheel_radius;
  nh.getParam("wheel_base",wheel_base);
  nh.getParam("wheel_radius",wheel_radius);

  diffa= DiffDrive(p,wheel_base,wheel_radius);
  send_TF(p, current_time);
  pub_odm (p,tw, current_time);

  return 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometer");

  string joint_state_topic;
  ros::NodeHandle nh1("~");
  ros::NodeHandle nh;
  nh1.getParam("joint_state_topic", joint_state_topic);

  odm_publisher = nh.advertise<nav_msgs::Odometry>("odom", 1000);
  joint_state_subscriber = nh.subscribe(joint_state_topic, 1000, poseCallback);
  ros::ServiceServer service = nh.advertiseService("/set_pose", set_pose_callback);


  string odm_frame_id;
  string body_frame_id;
  string left_wheel_joint;
  string right_wheel_joint;

  nh1.getParam("odom_frame_id",odom_frame_id);
  nh1.getParam("body_frame_id",body_frame_id);
  // nh1.getParam("body_frame_id",body_frame_id);
  nh1.getParam("left_wheel_joint",left_wheel_joint);
  nh1.getParam("right_wheel_joint",right_wheel_joint);
  nh1.getParam("base_link",base_link);

  double wheel_base;
  double wheel_radius;
  nh.getParam("wheel_base",wheel_base);
  nh.getParam("wheel_radius",wheel_radius);

  //
  Pose pp1;
  pp1.x = 0;
  pp1.y = 0;
  pp1.theta = 0;
  // DiffDrive diffa;
  diffa= DiffDrive(pp1,wheel_base,wheel_radius);
  //
  //
  WheelVelocities wheel_v;
  Pose P;
  double duration;
  ros::Time current_time;
  ros::Time last_time;
  current_time = ros::Time::now();
  last_time = current_time;

  while(ros::ok()){
    ros::spinOnce();
    current_time = ros::Time::now();
    duration = (current_time - last_time).toSec();
    last_time = current_time;
    Twist2D tw;
    wheel_v = wheel_vel();

    tw = diffa.wheelsToTwist(wheel_v);
    diffa.feedforward(tw,duration);
    P = diffa.pose();
    send_TF(P, current_time);
    pub_odm (P,tw, current_time);

  }

  return 0;
}

void poseCallback(const sensor_msgs::JointState::ConstPtr & mesg){
	wheel1.name = mesg->name[0];
  wheel2.name = mesg->name[1];
  wheel1.position = mesg->position[0];
  wheel2.position = mesg->position[1];
  wheel2.velocity = mesg->velocity[1];
  wheel1.velocity = mesg->velocity[0];
}


WheelVelocities wheel_vel(){
  WheelVelocities w;
  w.u1 = wheel1.velocity;
  w.u2 = wheel2.velocity;
  w.u3 = w.u2;
  w.u4 = w.u1;
  return w;
}

void send_TF(Pose P,ros::Time current_time){
  static tf2_ros::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = odom_frame_id;
  odom_trans.child_frame_id = base_link;
  odom_trans.transform.translation.x = P.x;
  odom_trans.transform.translation.y = P.y;
  odom_trans.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, P.theta);
  odom_trans.transform.rotation.x = q.x();
  odom_trans.transform.rotation.y = q.y();
  odom_trans.transform.rotation.z = q.z();
  odom_trans.transform.rotation.w = q.w();

  odom_broadcaster.sendTransform(odom_trans);
}

void pub_odm (Pose P,Twist2D tw,ros::Time current_time){
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(P.theta);
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = odom_frame_id;
  //set the position
  odom.pose.pose.position.x = P.x;
  odom.pose.pose.position.y = P.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

    //set the velocity
  odom.child_frame_id = base_link;
  odom.twist.twist.linear.x = tw.vx;
  odom.twist.twist.linear.y = tw.vy;
  odom.twist.twist.angular.z = tw.theta_dot;
  odm_publisher.publish(odom);
}
