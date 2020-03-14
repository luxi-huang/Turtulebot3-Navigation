// real_waypoints: enable the turtlebot to follow the trajectory of a rectangle
//
// PUBLISHERS:
//   + turtle1/cmd_vel (Twist) ~ publish turtle velocity.
//   + visualization_msgs/Marker (visualization_marker) - pubslish marker

// SUBSCRIBERS:
//   + nav_msgs::Odometry (odom) ~ subscribe the current pose of the turtlebot

#include "ros/ros.h"
#include <sstream>
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "std_srvs/Empty.h"
#include <cstdlib>
#include <tsim/PoseError.h>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "rigid2d/set_pen.h"
#include "nuturtle_robot/start.h"
// using namespace std; // use for count

using rigid2d::Vector2D;
using rigid2d::Waypoints;
using rigid2d::Velocity;
using rigid2d::Pose;
using rigid2d::Twist2D;
using rigid2d::DiffDrive;


ros::Publisher velocity_publisher;
ros::Publisher marker_pub;
ros::Subscriber odomSub;
ros::ServiceServer service;

void publish_cmd(Twist2D ttwist);
void publish_marker(Pose p_pose, visualization_msgs::Marker marker, int number);
void publish_zero_vel();
int set_pose_client();
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

uint32_t shape = visualization_msgs::Marker::CUBE;
Pose pose_odom;
int server_value = 0;

bool start_callback(nuturtle_robot::start::Request  &req,
                  nuturtle_robot::start::Response &resp)
{
	// set_pose_client();

	if (req.clockwise_forward == 1){
		server_value = 1;
		ROS_INFO("start_service111");
  	return 1;
	} else {
		server_value = 1;
		ROS_INFO("start_service000");
		return 1;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_way");
	ros::NodeHandle n;
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	odomSub  = n.subscribe<nav_msgs::Odometry>("/odom", 10, odomCallback);
	service = n.advertiseService("/start", start_callback);

	int trans_vel;
	int rot_vel;
	int frequency;

	n.getParam("trans_vel",trans_vel);
	n.getParam("rot_vel",rot_vel);
	n.getParam("frequency",frequency);

	ROS_INFO ("trans_vel :%d, The translational velocity of the robot",trans_vel);
	ROS_INFO ("rot_vel :%d, The rotational velocity of the robot",rot_vel);
	ROS_INFO ("frequency :%d, The frequency of the control loop",frequency);


	double frac_vel = 0.5;
	double maximum_translational_velocity = 0.22;
	double maximum_rotational_velocity_robot = 2.84;
	n.getParam("frac_vel",frac_vel);
	n.getParam("maximum_translational_velocity", maximum_translational_velocity);
	n.getParam("maximum_rotational_velocity_robot", maximum_rotational_velocity_robot);
	double translation_velocity = frac_vel*maximum_translational_velocity;
	double rotation_velocity = 0.1*maximum_rotational_velocity_robot;

	// initial waypoints;
	std::vector<Vector2D> p;
	// p = {{3.0,2.0},{7.0,3.0},{9.0,7.0},{5.0,10.0},{1.0,6.0}};
	p = {{0.0,0.0},{1.0,0.0},{1.5,0.5},{0.5,1.0},{-0.5,0.5}};
	trans_vel =2;
	rot_vel =1;

	Velocity vel;
	vel.linear = translation_velocity;
	vel.angular = rotation_velocity;
	// ROS_INFO("vel_linear~~~ %f", vel.linear);

	Waypoints way;
	way = Waypoints(p,vel);
	int goal = way.print_goal();

	double linear_threshold = 0.05;
	double angular_threshold = 0.01;
	int count =0;

	Pose init_pose;
	init_pose.x = p[0].x;
	init_pose.y = p[0].y;
	init_pose.theta = 0.0;
	DiffDrive check_error_diff;
	check_error_diff=DiffDrive(init_pose,0.0,0.0);

	while (ros::ok()){
		ros::spinOnce();
		if (server_value == 1)
		{
			set_pose_client();
			double duration;
			ros::Time current_time;
			ros::Time last_time;
			current_time = ros::Time::now();
			last_time = current_time;

			int frequency = 20;
			ros::Rate r(frequency);
	    while (1){
	      ros::spinOnce();
				Pose estimate_pose;
				// check_error_diff=DiffDrive(pose_odom,0.0,0.0);
				estimate_pose = check_error_diff.pose();

				double distance_to_goal = way.left_distance(estimate_pose);
				double angle_to_goal = way.left_angle(estimate_pose);

				Twist2D twist;
				int last_goal = way.print_goal();
				twist = way.nextWaypoint(distance_to_goal, angle_to_goal,linear_threshold, angular_threshold);
				int current_goal = way.print_goal();
				// if (last_goal != current_goal){
				// 	check_error_diff=DiffDrive(pose_odom,0.0,0.0);
				// }
				count++;
				if (count > 100){
					count = 0;
					check_error_diff=DiffDrive(pose_odom,0.0,0.0);
				}

				if(last_goal == 0 && current_goal == 1){
					while(1)
					{
						publish_zero_vel();
						ros::spinOnce();
					}
				}

				// ROS_INFO("twist_ x  %f ", twist.vx);

				publish_cmd(twist);

				current_time = ros::Time::now();
				duration = (current_time - last_time).toSec();
				last_time = current_time;
				check_error_diff.feedforward(twist,duration);

				// set_marker;
				Pose p_pose1;
				p_pose1.x = p[0].x;
				p_pose1.y = p[0].y;
				p_pose1.theta = 0.0;

				Pose p_pose2;
				p_pose2.x = p[1].x;
				p_pose2.y = p[1].y;
				p_pose2.theta = 0.0;

				Pose p_pose3;
				p_pose3.x = p[2].x;
				p_pose3.y = p[2].y;
				p_pose3.theta = 0.0;

				Pose p_pose4;
				p_pose4.x = p[3].x;
				p_pose4.y = p[3].y;
				p_pose4.theta = 0.0;

				Pose p_pose5;
				p_pose5.x = p[4].x;
				p_pose5.y = p[4].y;
				p_pose5.theta = 0.0;

				visualization_msgs::Marker marker1;
				visualization_msgs::Marker marker2;
				visualization_msgs::Marker marker3;
				visualization_msgs::Marker marker4;
				visualization_msgs::Marker marker5;

				publish_marker(p_pose1, marker1, 1);
				publish_marker(p_pose2, marker2, 2);
				publish_marker(p_pose3, marker3, 3);
				publish_marker(p_pose4, marker4, 4);
				publish_marker(p_pose5, marker5, 5);



				r.sleep();
	    }
		} // if loop close
	}
}


void publish_cmd(Twist2D ttwist){
	geometry_msgs::Twist goal_msg;
	goal_msg.linear.x = ttwist.vx;
	goal_msg.linear.y = ttwist.vy;
	goal_msg.linear.z =0;
	goal_msg.angular.x = 0;
	goal_msg.angular.y = 0;
	goal_msg.angular.z = ttwist.theta_dot;
	velocity_publisher.publish(goal_msg);
}

void publish_zero_vel(){
	geometry_msgs::Twist goal_msg;
	goal_msg.linear.x = 0;
	goal_msg.linear.y = 0;
	goal_msg.linear.z =0;
	goal_msg.angular.x = 0;
	goal_msg.angular.y = 0;
	goal_msg.angular.z = 0;
	velocity_publisher.publish(goal_msg);
}


void publish_marker(Pose p_pose,visualization_msgs::Marker marker, int number){
  // visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/odom_frame_id";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = number;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = p_pose.x;
  marker.pose.position.y = p_pose.y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);

}


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	pose_odom.x = msg->pose.pose.position.x;
	pose_odom.y = msg->pose.pose.position.y;

	tf::Quaternion q(
		 msg->pose.pose.orientation.x,
		 msg->pose.pose.orientation.y,
		 msg->pose.pose.orientation.z,
		 msg->pose.pose.orientation.w );

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	pose_odom.theta = yaw;
	// pub_pose_.publish(pose2d);
}

int set_pose_client(){
	ros::NodeHandle n;
	ros::ServiceClient client= n.serviceClient<rigid2d::set_pen>("/set_pose");
  rigid2d::set_pen Set_pose_srv;
  Set_pose_srv.request.robot_pose.x = 0;
	Set_pose_srv.request.robot_pose.y = 0;
	Set_pose_srv.request.robot_pose.theta = 0;

	ros::service::waitForService("/set_pose");
	if (client.call(Set_pose_srv))
		{
			ROS_INFO("set_pose_called");
		}
		else
		{
			ROS_ERROR("Failed to call service set_pose");
			return 1;
		}
	return 0;
}
