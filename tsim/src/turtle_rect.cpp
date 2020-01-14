#include "ros/ros.h"
#include <sstream>
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "std_srvs/Empty.h"

using namespace std; // use for count

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
// ros::NodeHandle n;

void go_stright(double speed, double distance, bool ifForward);
void rotate (double angular_vel, double relative_rotate_angle_degree, bool clockwise);
void set_relative_angle(double desired_angle_degree);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void go_to_goal(turtlesim::Pose  goal_pose, double distance_tolerance);
int turtle_setpen_client(int off);
int turtle_teleport_client(float x_coordinate, float y_coordinate, float theta);
int traj_rest_client();


bool traj_reset(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{

	int x_value;
	int y_value;
	ros::NodeHandle n;
	n.getParam("x",x_value);
	n.getParam("y",y_value);
	turtlesim::Pose pose;
	pose.x = x_value;
	pose.y = y_value;
	pose.theta=0;
	go_to_goal(pose,0.001);
	return true;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_rect");
	ros::NodeHandle n;
	publich: n;
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
	ros::ServiceServer service = n.advertiseService("/trajectory_reset",traj_reset);


	// get parameter
	int x_value;
	int y_value;
	int width;
	int height;
	int trans_vel;
	int rot_vel;
	int frequency;

	n.getParam("x",x_value);
	n.getParam("y",y_value);
	n.getParam("width",width);
	n.getParam("height",height);
	n.getParam("trans_vel",trans_vel);
	n.getParam("rot_vel",rot_vel);
	n.getParam("frequency",frequency);

	ROS_INFO ("x :%d, The x coordinate of the lower left corner of a rectangle",x_value);
	ROS_INFO ("y :%d, The y coordinate of the lower left corner of a rectangle",y_value);
	ROS_INFO ("width :%d, The width of the rectangle",width);
	ROS_INFO ("height :%d, The height of the rectangle",height);
	ROS_INFO ("trans_vel :%d, The translational velocity of the robot",trans_vel);
	ROS_INFO ("rot_vel :%d, The rotational velocity of the robot",rot_vel);
	ROS_INFO ("frequency :%d, The frequency of the control loop",frequency);

	turtle_setpen_client(1); // setpen off
	turtle_teleport_client(x_value, y_value, 0);
	turtle_setpen_client(0); // setpen off
  ros::Duration(1).sleep();

		// traj_rest_client();
	do{
		// turtle_setpen_client(1); // setpen off
		// turtle_teleport_client(x_value, y_value, 0);
		// turtle_setpen_client(0); // setpen off
		go_stright(trans_vel,width,true);
		rotate(rot_vel,90.0,false);
		go_stright(trans_vel,height,true);
		rotate(rot_vel,90.0,false);
		go_stright(trans_vel,width,true);
		rotate(rot_vel,90.0,false);
		go_stright(trans_vel,height,true);
		ros::Duration(1).sleep();
		traj_rest_client();
		rotate(rot_vel,90.0,false);

	}while(1);


	ros::spin();
}

// go straight function,input is speed, distance and direction
void go_stright(double speed, double distance, bool ifForward){
	geometry_msgs::Twist go_stright_msg;

	if (ifForward)
		go_stright_msg.linear.x =abs(speed);
	else
		go_stright_msg.linear.x =-abs(speed);

	go_stright_msg.linear.y =0;
	go_stright_msg.linear.z =0;
	go_stright_msg.angular.x = 0;
	go_stright_msg.angular.y = 0;
	go_stright_msg.angular.z =0;

	double start_time = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);

	//if current_distanace samller than desired distance, keep moving straight
	do{
		// double current_time = ros::Time::now().toSec();
		velocity_publisher.publish(go_stright_msg);
		double current_time = ros::Time::now().toSec();
		current_distance = speed * (current_time-start_time);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_distance<distance);

	// after arrived,set the velocity equal to immediently
	go_stright_msg.linear.x =0;
	velocity_publisher.publish(go_stright_msg);

}

// turtle_rotate function is for rotate the turtle to relative_angle with direction
void rotate (double angular_vel, double relative_rotate_angle_degree, bool clockwise){
  double relative_rotate_angle_radius = relative_rotate_angle_degree * M_PI /180.0;
	// double relative_rotate_angle_radius;
	// initial linear and angular velocity with all directions for 0
	geometry_msgs::Twist angular_msg;
	angular_msg.linear.x =0;
	angular_msg.linear.y =0;
	angular_msg.linear.z =0;
	angular_msg.angular.x = 0;
	angular_msg.angular.y = 0;
	angular_msg.angular.z = 0;

	// relative_rotate_angle_radius = relative_rotate_angle * M_PI /180.0;
	if (clockwise)
		angular_msg.angular.z =-abs(angular_vel);
	else
		angular_msg.angular.z =abs(angular_vel);

	double rotate_angle = 0.0;
	double start_time = ros::Time::now().toSec();

	// keep rotate until the rotate angle equal to relative_rotate_angle
	ros::Rate loop_rate(1000);
	do{
	  // double current_time = ros::Time::now().toSec();
		velocity_publisher.publish(angular_msg);
		double current_time = ros::Time::now().toSec();
		rotate_angle = angular_vel * (current_time-start_time);
		ros::spinOnce();
		loop_rate.sleep();
		// ROS_INFO("%f", rotate_angle);
	}while(rotate_angle<relative_rotate_angle_radius);

	// after reach to desired angle, set angle to 0
	angular_msg.angular.z =0;
	velocity_publisher.publish(angular_msg);

}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

void set_relative_angle (double desired_angle_degree){
	double desired_angle_radians = desired_angle_degree * M_PI /180.0;
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	double relative_angle_degree =relative_angle_radians *180.0/M_PI;
	bool clockwise = ((relative_angle_degree<0)?true:false);
	rotate (0.2, abs(relative_angle_degree), clockwise);

}

void go_to_goal(turtlesim::Pose  goal_pose, double distance_tolerance){

	geometry_msgs::Twist goal_msg;
	double left_distance;
	ros::Rate loop_rate(100);
	do{
		goal_msg.linear.x = 1*sqrt(pow((turtlesim_pose.x-goal_pose.x),2)+pow((turtlesim_pose.y-goal_pose.y),2));
		goal_msg.linear.y =0;
		goal_msg.linear.z =0;

		goal_msg.angular.x = 0;
		goal_msg.angular.y = 0;
		goal_msg.angular.z =2*(atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x-turtlesim_pose.x)-turtlesim_pose.theta);

		velocity_publisher.publish(goal_msg);

		ros::spinOnce();
		loop_rate.sleep();
		left_distance = sqrt(pow((turtlesim_pose.x-goal_pose.x),2)+pow((turtlesim_pose.y-goal_pose.y),2));
	}while(left_distance>distance_tolerance);
	goal_msg.linear.x =0;
	goal_msg.angular.z = 0;
	velocity_publisher.publish(goal_msg);
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
			ROS_INFO("Sum");
		}
		else
		{
			ROS_ERROR("Failed to call service set_pen");
			return 1;
		}
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
			ROS_INFO("Sum");
		}
		else
		{
			ROS_ERROR("Failed to call service teleport");
			return 1;
		}
	return 0;
}

// bool traj_reset(std_srvs::Empty::Request  &req,
// 								std_srvs::Empty::Response &res)
// {
// 	turtlesim::Pose pose;
// 	pose.x=req.x;
// 	pose.y=req.y;
// 	pose.theta=req.theta;
// 	go_to_goal(pose,0.01);
// 	return true;
// }


int traj_rest_client(){
	ros::NodeHandle n;
	ros::ServiceClient traj_reset_client= n.serviceClient<std_srvs::Empty>("/trajectory_reset");
  std_srvs::Empty traje_reset_srv;
	ros::service::waitForService("/trajectory_reset");
	return 0;
	// if (traj_rest_client.call(traje_reset_srv))
	// 	{
	// 		ROS_INFO("Sum");
	// 	}
	// 	else
	// 	{
	// 		ROS_ERROR("Failed to call service teleport");
	// 		return 1;
	// 	}
	// return 0;
}
