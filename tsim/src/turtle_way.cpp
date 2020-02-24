// turtle_rect: enable the turtle to follow the trajectory of
// a rectangle
//
// PUBLISHERS:
//   + turtle1/cmd_vel (Twist) ~ publish turtle velocity.
// 	+ pose_error (Pose_error) ~ publish the pose error of turtle.
//
// SUBSCRIBERS:
//   + turtle1/pose (Pose) ~ subscribe the current pose of the turtle1 turtle (x, y, theta)

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
// using namespace std; // use for count

using rigid2d::Vector2D;
using rigid2d::Waypoints;
using rigid2d::Velocity;
using rigid2d::Pose;
using rigid2d::Twist2D;
using rigid2d::DiffDrive;
// using rigid2d::Waypoints::left_distance;


ros::Publisher velocity_publisher;
// ros::Subscriber pose_subscriber;
// turtlesim::Pose turtlesim_pose;
// ros::Publisher PoseError_publisher;

// turtlesim::Pose  last_pose;
// ros::NodeHandles n;

// void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void publish_cmd(Twist2D ttwist);
// void publish_error(Pose estimate_pose);
// int turtle_setpen_client(int off);
// int traj_rest_client();
// int turtle_setpen_client(int off);
// int traj_rest_client();
// int Error_pose(int x, int y, int theta);
// void go_to_goal(turtlesim::Pose  goal_pose, double distance_tolerance, turtlesim::Pose &last_pose);
// double normalize_angle(double rad);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_way");
	ros::NodeHandle n;
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
	// PoseError_publisher = n.advertise<tsim::PoseError>("pose_error", 60);
	// pose_subscriber = n.subscribe("/turtle1/pose", 1, poseCallback);
  // std::vector<double> waypoints_x =  {3,7,9,5,1};
  // std::vector<double> waypoints_y =  {2,3,7,10,6};


	int trans_vel;
	int rot_vel;
	int frequency;

	n.getParam("trans_vel",trans_vel);
	n.getParam("rot_vel",rot_vel);
	n.getParam("frequency",frequency);

	ROS_INFO ("trans_vel :%d, The translational velocity of the robot",trans_vel);
	ROS_INFO ("rot_vel :%d, The rotational velocity of the robot",rot_vel);
	ROS_INFO ("frequency :%d, The frequency of the control loop",frequency);

	// initial waypoints;
	std::vector<Vector2D> p;
	p = {{3.0,2.0},{7.0,3.0},{9.0,7.0},{5.0,10.0},{1.0,6.0}};
	// p = {{0.0,0.0},{2.0,0.0},{3,2},{1.0,3.0},{-1.0,2.0}};
	trans_vel =2;
	rot_vel =1;
	
	Velocity vel;
	vel.linear = trans_vel;
	vel.angular = rot_vel;
	ROS_INFO("vel_linear~~~ %f", vel.linear);

	Waypoints way;
	way = Waypoints(p,vel);
	int goal = way.print_goal();
	// ROS_INFO("goal~~~ %d", goal);

	double linear_threshold = 0.1;
	double angular_threshold = 0.1;

	Pose init_pose;
	init_pose.x = p[0].x;
	init_pose.y = p[0].y;
	init_pose.theta = 0.0;
	DiffDrive check_error_diff;
	check_error_diff=DiffDrive(init_pose,0.0,0.0);

	while (ros::ok()){
		// ros::Duration(1).sleep();
		// traj_rest_client();
		// ros::Duration(1).sleep();
		// turtle_setpen_client(0);

		double duration;
		ros::Time current_time;
		ros::Time last_time;
		current_time = ros::Time::now();
		last_time = current_time;

		// TODO: change
		int frequency = 100;
		ros::Rate r(frequency);
    while (1){
      ros::spinOnce();
			Pose estimate_pose;
			estimate_pose = check_error_diff.pose();
			// publish_error(estimate_pose);

			// find the left distance and angle;
			// Pose pp;
			// pp.x = turtlesim_pose.x;
			// pp.y = turtlesim_pose.y;
			// pp.theta = turtlesim_pose.theta;
			double distance_to_goal = way.left_distance(estimate_pose);
			// ROS_INFO("distance_to_goal %f", distance_to_goal);
			double angle_to_goal = way.left_angle(estimate_pose);
			// ROS_INFO("left_distance %f", distance_to_goal);
			// ROS_INFO("left_angle %f ", angle_to_goal);
			// ROS_INFO_STREAM("rest_angle - angular threshold"<<angle_to_goal - angular_threshold);

			Twist2D twist;
			// if (std::abs(angle_to_goal)<=angular_threshold){
			// 	ROS_INFO("-------------");
			// 	ROS_INFO("angular threshold %f", angular_threshold);
			// 	ROS_INFO("angle_to_goal %f", angle_to_goal);
			// 	ROS_INFO("abs angle_to_goal %f", abs(angle_to_goal));
			// 	double lol = std::abs(angle_to_goal)-angular_threshold;
			// 	ROS_INFO("angle difference %f", lol);
			// }
			twist = way.nextWaypoint(distance_to_goal, angle_to_goal,linear_threshold, angular_threshold);
			int goal = way.print_goal();
			// ROS_INFO("goal!!! %d", goal);
			ROS_INFO("twist_ x  %f ", twist.vx);
			// ROS_INFO("twist_ y  %f ", twist.vy);
			// ROS_INFO("twist_ theta  %f ", twist.theta_dot);
			publish_cmd(twist);

			current_time = ros::Time::now();
			duration = (current_time - last_time).toSec();
			last_time = current_time;
			check_error_diff.feedforward(twist,duration);

			// Pose estimate_pose;
			// estimate_pose = check_error_diff.pose();
			// publish_error(estimate_pose);

			r.sleep();
    }
	}
}
//
// void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
// 	turtlesim_pose.x=pose_message->x;
// 	turtlesim_pose.y=pose_message->y;
// 	turtlesim_pose.theta=pose_message->theta;
// }

void publish_cmd(Twist2D ttwist){
	geometry_msgs::Twist goal_msg;
	goal_msg.linear.x = ttwist.vx;
	goal_msg.linear.y = ttwist.vy;
	goal_msg.linear.z =0;
	goal_msg.angular.x = 0;
	goal_msg.angular.y = 0;
	goal_msg.angular.z = ttwist.theta_dot;

	// ROS_INFO("goal_msg.linear.x %f ", goal_msg.linear.x);
	// ROS_INFO("goal_msg.angular.z  %f ", goal_msg.angular.z);
	ROS_INFO("publish_twist_ x  %f ", ttwist.vx);
	// ROS_INFO("publish_twist_ y  %f ", ttwist.vy);
	// ROS_INFO("publish_twist_ theta  %f ", ttwist.theta_dot);
	// ROS_INFO("-------------------");

	velocity_publisher.publish(goal_msg);
}

// void publish_error(Pose estimate_pose){
// 	tsim::PoseError p_error;
// 	p_error.x_error = turtlesim_pose.x - estimate_pose.x;
// 	p_error.y_error = turtlesim_pose.y - estimate_pose.y;
// 	p_error.theta_error = turtlesim_pose.theta - estimate_pose.theta;
// 	PoseError_publisher.publish(p_error);
// }
//
// int traj_rest_client(){
// 	ros::NodeHandle n;
//   ros::ServiceClient client= n.serviceClient<std_srvs::Empty>("turtle1/trajectory_reset", 1000);
//   std_srvs::Empty srv;
//   ros::service::waitForService("turtle1/trajectory_reset");
//   if (client.call(srv))
//           {
//                   ROS_INFO("CALL RESET");
//           }
//           else
//           {
//                   ROS_ERROR("Failed to call service teleport");
//                   return 1;
//           }
//   return 0;
// }
//
//
// int turtle_setpen_client(int off){
// 	ros::NodeHandle n;
// 	ros::ServiceClient client= n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
//   turtlesim::SetPen SetPen_srv;
//   SetPen_srv.request.r = 0;
// 	SetPen_srv.request.g = 0;
// 	SetPen_srv.request.b = 0;
// 	SetPen_srv.request.width = 1;
// 	SetPen_srv.request.off = off;
//
// 	ros::service::waitForService("turtle1/set_pen");
// 	if (client.call(SetPen_srv))
// 		{
// 			ROS_INFO("Sum");
// 		}
// 		else
// 		{
// 			ROS_ERROR("Failed to call service set_pen");
// 			return 1;
// 		}
// 	return 0;
// }
