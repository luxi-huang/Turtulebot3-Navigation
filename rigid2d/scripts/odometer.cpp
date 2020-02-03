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

using namespace std; // use for count
using namespace rigid2d;

ros::Publisher odm_publisher;
ros::Subscriber joint_state_subscriber;
// tf::TransformBroadcaster odom_broadcaster;

tf2_ros::TransformBroadcaster odom_broadcaster;
sensor_msgs::JointState wheel1,wheel2;
void poseCallback(const sensor_msgs::JointState::ConstPtr & mesg);
WheelVelocities wheel_vel();
void send_TF(Pose P,ros::Time current_time);
void pub_odm (Pose P,Twist2D tw,ros::Time current_time);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometer");

  ros::NodeHandle nh1;

  string odm_frame_id;
  string body_frame_id;
  nh1.getParam("odm_frame_id",odm_frame_id);
  nh1.getParam("body_frame_id",body_frame_id);
  // n.getParam();
  double wheel_base;
  double wheel_radius;
  nh1.getParam("wheel_base",wheel_base);
  nh1.getParam("wheel_radius",wheel_radius);


  Pose pp;
  pp.x = 0;
  pp.y = 0;
  pp.theta = 0;
  wheel_base = 4.0;
  wheel_radius =2.0;
  DiffDrive diff;
  diff= DiffDrive(pp,wheel_base,wheel_radius);
  odm_publisher = nh1.advertise<nav_msgs::Odometry>("Odometry", 1000);
  joint_state_subscriber = nh1.subscribe("joint_state", 10,poseCallback);

  ros::Time current_time;

  while(ros::ok()){
    Twist2D tw;
    WheelVelocities wheel_v;
    Pose P;
    wheel_v = wheel_vel();
    tw = diff.wheelsToTwist(wheel_v);
    diff.feedforward(tw);
    P = diff.pose();
    current_time = ros::Time::now();
    send_TF(P, current_time);
    pub_odm (P,tw, current_time);
    ROS_INFO("nice!!!");
    ros::spinOnce();
  }

  return 0;
}

void poseCallback(const sensor_msgs::JointState::ConstPtr & mesg){
	wheel1.name[0] = mesg->name[0];
  wheel2.name[0] = mesg->name[1];
  wheel1.position[0] = mesg->position[0];
  wheel2.position[0] = mesg->position[1];
  wheel1.velocity[0] = mesg->velocity[0];
  wheel2.velocity[0] = mesg->velocity[1];
}


WheelVelocities wheel_vel(){
  WheelVelocities w;
  w.u1 = wheel1.velocity[0];
  w.u2 = wheel2.velocity[0];
  w.u3 = w.u2;
  w.u4 = w.u1;
  return w;
}
//
void send_TF(Pose P,ros::Time current_time){
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom_frame_id";
  odom_trans.child_frame_id = "body_frame_id";
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
  odom.header.frame_id = "odom_frame_id";
  //set the position
  odom.pose.pose.position.x = P.x;
  odom.pose.pose.position.y = P.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

    //set the velocity
  odom.child_frame_id = "body_frame_id";
  odom.twist.twist.linear.x = tw.vx;
  odom.twist.twist.linear.y = tw.vy;
  odom.twist.twist.angular.z = tw.theta_dot;
  odm_publisher.publish(odom);
}
