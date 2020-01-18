// """
// Reset_sever: set the std_srvs service.
//
// SERVICE:
// + turtle1/trajectory_reset (reset turtle to the left coner of trajectory)
// """


#include "ros/ros.h"
#include <sstream>
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "std_srvs/Empty.h"
#include <cstdlib>

int turtle_teleport_client(float x_coordinate,float y_coordinate,float theta);
int turtle_setpen_client(int off);

bool callback(std_srvs::Empty::Request  &req,
								std_srvs::Empty::Response &resp)
{
	ros::NodeHandle n;
	int x_value;
	int y_value;
	n.getParam("x",x_value);
	n.getParam("y",y_value);
	turtle_setpen_client(1);
	turtle_teleport_client(x_value, y_value, 0);
	ros::spinOnce();
	// ROS_INFO("11111111");
	return 1;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reset_sever");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("turtle1/trajectory_reset",callback);
	ROS_INFO ("ready to call service");
	ros::spin();

	return 0;
}

int turtle_teleport_client(float x_coordinate,float y_coordinate,float theta){
	ros::NodeHandle n;
	ros::ServiceClient teleport_client= n.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
  turtlesim::TeleportAbsolute teleport_srv;
	teleport_srv.request.x = x_coordinate;
	teleport_srv.request.y = y_coordinate;
	teleport_srv.request.theta = theta;

	ros::service::waitForService("turtle1/teleport_absolute");
	if (teleport_client.call(teleport_srv))
		{
			ROS_INFO("CALL TELEPORT");
		}
		else
		{
			ROS_ERROR("Failed to call service teleport");
			return 1;
		}
	return 0;
}

int turtle_setpen_client(int off){
	ros::NodeHandle n;
	ros::ServiceClient client= n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
  turtlesim::SetPen SetPen_srv;
  SetPen_srv.request.r = 0;
	SetPen_srv.request.g = 0;
	SetPen_srv.request.b = 0;
	SetPen_srv.request.width = 1;
	SetPen_srv.request.off = off;

	ros::service::waitForService("turtle1/set_pen");
	if (client.call(SetPen_srv))
		{
			ROS_INFO("CALL SetPen");
		}
		else
		{
			ROS_ERROR("Failed to call service set_pen");
			return 1;
		}
	return 0;
}
