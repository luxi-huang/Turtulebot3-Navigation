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
using namespace std; // use for count

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
ros::Publisher PoseError_publisher;
// ros::NodeHandles n;

void go_stright(double speed, double distance, bool ifForward,double xvalue, double yalue, int dirct);
void rotate (double angular_vel, double relative_rotate_angle_degree, bool clockwise);
void set_relative_angle(double desired_angle_degree);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
int turtle_setpen_client(int off);
int traj_rest_client();
int Error_pose(int x, int y, int theta);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_rect");
	ros::NodeHandle n;
	// publich:: n;
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	PoseError_publisher = n.advertise<tsim::PoseError>("pose_error", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

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

	ros::Rate loop_rate(1000);
	while (ros::ok()){
		ros::spinOnce();
		ros::Duration(2).sleep();
		traj_rest_client();
		ros::Duration(2).sleep();
		turtle_setpen_client(0);
		go_stright(trans_vel,width,true,x_value, y_value, 0);
		// Error_pose(x_value+width, y_value, 0);
		rotate(rot_vel,90.0,false);
		go_stright(trans_vel,height,true,x_value+width, y_value, 90);
		// Error_pose(x_value+width, y_value+height, 90);
		rotate(rot_vel,90.0,false);
		go_stright(trans_vel,width,true,x_value+width, y_value+height, 180);
		// Error_pose(x_value, y_value+height, 180);
		rotate(rot_vel,90.0,false);
		go_stright(trans_vel,height,true,x_value, y_value+height,-90);
		// Error_pose(x_value, y_value+height, -90);
	}
}

void go_stright(double speed, double distance, bool ifForward,double xvalue, double yvalue, int direct){
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

		if (direct == 0) {
			Error_pose(current_distance + xvalue, yvalue, direct);
		}else if (direct == 90){
			Error_pose(xvalue, current_distance + yvalue, direct);
		}else if (direct == -180){
			Error_pose(xvalue - current_distance, yvalue, direct);
		}else if (direct == -90){
			Error_pose(xvalue, yvalue-current_distance, direct);
		}




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

int traj_rest_client(){
	ros::NodeHandle n;
  ros::ServiceClient client= n.serviceClient<std_srvs::Empty>("turtle1/trajectory_reset", 1000);
  std_srvs::Empty srv;
  ros::service::waitForService("turtle1/trajectory_reset");
  if (client.call(srv))
          {
                  ROS_INFO("CALL RESET");
          }
          else
          {
                  ROS_ERROR("Failed to call service teleport");
                  return 1;
          }
  return 0;
}

int Error_pose(int x, int y, int theta){
	tsim::PoseError p_error;
	p_error.x_error = turtlesim_pose.x - x;
	p_error.y_error = turtlesim_pose.y - y ;
	theta = theta * M_PI/180;
	p_error.theta_error = turtlesim_pose.theta - theta;
	PoseError_publisher.publish(p_error);
	return 0;
}
