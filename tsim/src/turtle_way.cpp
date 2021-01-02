/// \file: 
/// \brief: This file has the turtle follow a trajectory of user specified waypoints  
///
/// PUBLISHERS:
///   + turtle1/cmd_vel (Twist) ~ publish turtle velocity.
///   + pose_error (Pose_error) ~ publish the pose error of turtle.
///
/// SUBSCRIBERS:
///   + turtle1/pose (Pose) ~ subscribe the current pose of the turtle1 turtle (x, y, theta)

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

using namespace std; // use for count

class TurtleWay 
{
public: 
	TurtleWay(ros::NodeHandle &nh) 
	{
		velocity_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",60);
		PoseError_pub = nh.advertise<tsim::PoseError>("pose_error", 60);
		pose_sub = nh.subscribe("/turtle1/pose", 1000, &TurtleWay::poseCallback, this);

		set_pen_client = nh.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
		teleport_turtle_client= nh.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
		
		// get waypoints from yaml file
		std::vector<double> waypoint_x;
        std::vector<double> waypoint_y;
		nh.getParam("/waypoint_x", waypoint_x);
        nh.getParam("/waypoint_y", waypoint_y);
		
		nh.getParam("trans_vel",trans_vel);
		nh.getParam("rot_vel",rot_vel);
		nh.getParam("frequency",frequency);
		nh.getParam("linear_threshold",linear_threshold);
		nh.getParam("angular_threshold",angular_threshold);

		// nh.getParam("/wheel_base", wheel_base_);
        // nh.getParam("/wheel_radius", wheel_radius_);

		// std::vector<rigid2d::Vector2D> waypoint;

		for (int i = 0; i < waypoint_x.size(); i++)
        {
            rigid2d::Vector2D inital_point(waypoint_x.at(i), waypoint_y.at(i));
            ways.push_back(inital_point);
        }

		// std::cout << "way_size: " << ways[0].x << "\n"; 

		rigid2d::Vector2D vec(ways[0].x,ways[0].y);
		rigid2d::Transform2D pose(vec, 0.0);
		rigid2d::DiffDrive my_diff(pose, 0.0, 0.0);

		if (rot_vel > 0.5) rot_vel = 0.5;
		if (trans_vel > 0.5) trans_vel = 0.5;
		rigid2d::Twist2D vel(rot_vel,trans_vel,0);
		traj_generator = rigid2d::Waypoints(ways, vel, my_diff, frequency, linear_threshold, angular_threshold);
	}

	void pipeline()
    {
		// lift pen and teleport turtle and put pen down. 	
		turtle_setpen_client(1);
		ros::Duration(2).sleep();
		Teleport_client();
		turtle_setpen_client(0);

		ros::Duration(1).sleep();
		ros::Rate rate(60);

		while (ros::ok())
        {
			traj_generator.update_state();
			rigid2d::Twist2D vel = traj_generator.nextWaypoint();
			
			// publish vel to turtle;
			geometry_msgs::Twist updateSpeed_msg;
			updateSpeed_msg.linear.x = vel.vx;
			updateSpeed_msg.angular.z = vel.theta_dot;
			velocity_pub.publish(updateSpeed_msg);
			
			traj_generator.update_pose(vel);
			double target_x, target_y, target_theta;
			tsim::PoseError error_msg;
			traj_generator.getPose(target_x, target_y, target_theta);
			error_msg.x_error = sub_x - target_x;
            error_msg.y_error = sub_y - target_y;
            error_msg.theta_error = rigid2d::normalize_angle(sub_theta - target_theta);
            PoseError_pub.publish(error_msg);
			rate.sleep();
			ros::spinOnce();

		}

	}

	void poseCallback(const turtlesim::Pose &msg)
    {
        // save the actual pose from the turtle simulator
        sub_x = msg.x;
        sub_y = msg.y;
        sub_theta = msg.theta;
    }

	int turtle_setpen_client(int off) 
	{	
		/*  Setup set pen survice;
		*	 Lift pen when input is true; */
		turtlesim::SetPen SetPen_srv;
		SetPen_srv.request.r = 0;
		SetPen_srv.request.g = 0;
		SetPen_srv.request.b = 0;
		SetPen_srv.request.width = 1;
		SetPen_srv.request.off = off;

		ros::service::waitForService("turtle1/set_pen");
		if (set_pen_client.call(SetPen_srv))
			{
				ROS_INFO("Pen Lifted");
			}
			else
			{
				ROS_ERROR("Failed to call service set_pen");
				return 1;
			}
		return 0;
	}	

	int Teleport_client() 
	{
		/*  Setup set pen survice;
		*	teleport turtle to the left corner of window; */
		turtlesim::TeleportAbsolute teleport_absolute_req;
		// teleport_absolute_req.request.x = ways.at(0).x;
		// teleport_absolute_req.request.y = ways.at(0).y;
		teleport_absolute_req.request.x = 1;
		teleport_absolute_req.request.y = 1;

		teleport_absolute_req.request.theta = 0.0;

		ros::service::waitForService("turtle1/teleport_absolute");
		if (teleport_turtle_client.call(teleport_absolute_req))
			{
				ROS_INFO("Teleport to left corner");
			}
			else
			{
				ROS_ERROR("Failed to call service teleport_absolute");
				return 1;
			}
		return 0;
	}
	

private:
	ros::Publisher velocity_pub;
	ros::Subscriber pose_sub;
	ros::Publisher PoseError_pub;
	ros::ServiceClient set_pen_client;
	vector<rigid2d::Vector2D> ways;
	ros::ServiceClient teleport_turtle_client;
	rigid2d::Waypoints traj_generator;
	double trans_vel, rot_vel, frequency, linear_threshold, angular_threshold;
	double sub_x, sub_y, sub_theta; 
};

int main(int argc, char **argv)
{
    // init the node
    ros::init(argc, argv, "turtle_way");
    ros::NodeHandle nh;
    TurtleWay turtle = TurtleWay(nh);
    turtle.pipeline();

    ros::spin();
    return 0;
}


