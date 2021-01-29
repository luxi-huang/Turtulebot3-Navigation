/// \file
/// \brief real_waypoints: enable the turtlebot to follow the trajectory of a rectangle
///
/// PUBLISHERS:
///   + turtle1/cmd_vel (Twist) ~ publish turtle velocity.
///   + visualization_msgs/Marker (visualization_marker) - pubslish marker
///
/// SUBSCRIBERS:
///   + nav_msgs::Odometry (odom) ~ subscribe the current pose of the turtlebot

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
#include "tf/transform_datatypes.h"
#include "rigid2d/set_pen.h"
#include "nuturtle_robot/start.h"
#include "nuturtle_robot/Stop.h"
using namespace std; // use for count

class RealWaypoint
{

public:
	RealWaypoint(ros::NodeHandle &nh)
    {
		// nh.getParam("trans_vel",trans_vel);
		// nh.getParam("rot_vel",rot_vel);
		// nh.getParam("frequency",frequency);

		nh.getParam("frac_vel",frac_vel);
		nh.getParam("maximum_translational_velocity", max_trans_vel);
		nh.getParam("maximum_rotational_velocity_robot", max_rot_vel);

		nh.getParam("/encoder_ticks", encoder_ticks);
        nh.getParam("/wheel_base", wheel_base);
        nh.getParam("/wheel_radius", wheel_radius);

		nh.getParam("linear_threshold",linear_threshold);
		nh.getParam("angular_threshold",angular_threshold);

		//service 
		start_service = nh.advertiseService("/start", &RealWaypoint::start_callback, this);
		odomSub  = nh.subscribe("/odom", 10, &RealWaypoint::odomCallback, this);
		stop_server = nh.advertiseService("stop", &RealWaypoint::stop_callback, this);

		velocity_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
		marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
		
		frequency = 110;

		std::vector<double> waypoint_x;
        std::vector<double> waypoint_y;
		nh.getParam("/waypoint_x", waypoint_x);
        nh.getParam("/waypoint_y", waypoint_y);

		for (int i = 0; i < waypoint_x.size(); i++)
        {
            rigid2d::Vector2D inital_point(waypoint_x.at(i), waypoint_y.at(i));
            ways.push_back(inital_point);
        }
		
		rigid2d::Vector2D vec(0, 0);
		rigid2d::Transform2D pose(vec, 0.0);
		rigid2d::DiffDrive my_diff(pose, wheel_base, wheel_radius);
		
		double translation_velocity = frac_vel * max_trans_vel;
		double rotation_velocity = frac_vel * max_rot_vel;

		rigid2d::Twist2D vel(rotation_velocity,translation_velocity,0);
		traj_generator = rigid2d::Waypoints(ways, vel, my_diff, frequency, linear_threshold, angular_threshold);

	}	

	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		current_x = msg->pose.pose.position.x;
		current_y = msg->pose.pose.position.y;

        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);

		double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

		current_theta = yaw;
	}	

	bool start_callback(nuturtle_robot::start::Request &req,
                  nuturtle_robot::start::Response &resp)
	{
		// set_pose_client();
		ros::Rate rate(frequency);
        for (int i = 0; i < ways.size(); i++)
        {
            publish_waypoint_marker(ways.at(i).x, ways.at(i).y, i);
            // wait marker publishing to finish
            ros::spinOnce();
            ros::Duration(0.2).sleep();
            // ROS_INFO("Published Marker at %f, %f;", waypoints_.at(i).x, waypoints_.at(i).y);
        }


	    while (ros::ok())
        {
			traj_generator.update_state();
			rigid2d::Twist2D vel = traj_generator.nextWaypoint();
						// publish vel to turtle;
			geometry_msgs::Twist updateSpeed_msg;
			updateSpeed_msg.linear.x = vel.vx;
			updateSpeed_msg.angular.z = vel.theta_dot;
			velocity_publisher.publish(updateSpeed_msg);

			traj_generator.update_pose_close_loop(current_x, current_y, current_theta);
			rate.sleep();
			ros::spinOnce();
		}

	}

	bool stop_callback(nuturtle_robot::Stop::Request &req,
                       nuturtle_robot::Stop::Response &res)
    {
        // stop the motor
        if_stop_ = true;
        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.angular.z = 0;
        velocity_publisher.publish(twist);

        return true;
    }

	int publish_waypoint_marker(double x, double y, int marker_id)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/odom";
        marker.header.stamp = ros::Time::now();

        marker.ns = "markers";
        marker.id = marker_id;

        uint32_t shape = visualization_msgs::Marker::CYLINDER;
        marker.type = shape;

        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        // Publish the marker
        marker_pub.publish(marker);
        ros::spinOnce();

        return 0;
    }




private:

	double max_trans_vel;
	double max_rot_vel;
	double frac_vel;
	// double max_mot_vel_;
	double encoder_ticks;
	double wheel_base;
	double wheel_radius;

	ros::ServiceServer start_service, stop_server;
	ros::Subscriber odomSub;
	ros::Publisher velocity_publisher, marker_pub;

	// int trans_vel;
	// int rot_vel;
	int frequency;
	vector<rigid2d::Vector2D> ways;
	rigid2d::Waypoints traj_generator;
	double linear_threshold, angular_threshold;

	bool if_stop_;
	double current_x, current_y;
	double current_theta;
};





int main(int argc, char **argv)
{
    // init the node
    ros::init(argc, argv, "real_waypoint");
    ros::NodeHandle nh;
    RealWaypoint my_real_waypoint = RealWaypoint(nh);

    ros::spin();
    return 0;
}