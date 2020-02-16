// """
// Reset_sever: set_pose
//
// SERVICE:
// + set_pen server to set the odometry pose of robot;
// """

#include "ros/ros.h"
#include <sstream>
#include <cstdlib>
#include "rigid2d/set_pen.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "rigid2d/diff_drive.hpp"

using rigid2d::Pose;
using rigid2d::Twist2D;

void send_TF(Pose P,ros::Time current_time);
void pub_odm (Pose P,Twist2D tw,ros::Time current_time);


ros::Publisher odm_publisher;

class Set_Pose{
	// Pose p;
	// Twist2D tw;
	// tw.vx = 0;
	// tw.vy = 0;
	// tw.theta_dot = 0;

public:
	Set_Pose(){
		ros::NodeHandle n;
		odm_publisher = n.advertise<nav_msgs::Odometry>("Odometry/Odometry", 1000);
		// ros::Time current_time;
		// send_TF(p, current_time);
		// pub_odm (p,tw, current_time);
	}



	bool callback(rigid2d::set_pen::Request  &req,
									rigid2d::set_pen::Response &resp)
	{
	  // ros::NodeHandle n;
	  // odm_publisher = n.advertise<nav_msgs::Odometry>("Odometry/Odometry", 1000);

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

	  send_TF(p, current_time);
	  pub_odm (p,tw, current_time);

		return 1;
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
	  odom.child_frame_id = "base_link";
	  odom.twist.twist.linear.x = tw.vx;
	  odom.twist.twist.linear.y = tw.vy;
	  odom.twist.twist.angular.z = tw.theta_dot;
	  odm_publisher.publish(odom);
	}

	void send_TF(Pose P,ros::Time current_time){
	  static tf2_ros::TransformBroadcaster odom_broadcaster;
	  geometry_msgs::TransformStamped odom_trans;
	  odom_trans.header.stamp = current_time;
	  odom_trans.header.frame_id = "odom_frame_id";
	  odom_trans.child_frame_id = "base_link";
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





};




int main(int argc, char **argv)
{
	ros::init(argc, argv, "set_pose_service");
	ros::NodeHandle n;
	Set_Pose sett;
	// odm_publisher = n.advertise<nav_msgs::Odometry>("Odometry/Odometry", 1000);
	ros::ServiceServer service = n.advertiseService("/set_pose",& Set_Pose::callback, &sett);
	ROS_INFO ("ready to call service");
	ros::spin();

	return 0;
}
