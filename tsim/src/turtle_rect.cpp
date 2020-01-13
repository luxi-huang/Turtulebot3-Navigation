#include "ros/ros.h"
#include <sstream>
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

using namespace std; // use for count

ros::Publisher velocity_publisher;
// ros::Subscriber pose_subscriber;
// turtlesim::Pose turtlesim_pose;

void go_stright(double speed, double distance, bool ifForward);
void turtle_rotate (double angular_vel, double relative_rotate_angle, bool clockwise);
// void setDesiredOrientation (double desired_angle_radians);
// void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
// void moveGoal(turtlesim::Pose  goal_pose, double distance_tolerance);
// void gridClean();
// void spiralClean();

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_rect");
	ros::NodeHandle n;

	// double speed, angular_speed;
	// double distance, angle;
	// bool ifForward, clockwise;


	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
  // go_stright(2.0, 9.0, true);
	turtle_rotate(0.2,90.0,true);

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
void turtle_rotate (double angular_vel, double relative_rotate_angle, bool clockwise){
  double relative_rotate_angle_radius;
	// initial linear and angular velocity with all directions for 0
	geometry_msgs::Twist angular_msg;
	angular_msg.linear.x =0;
	angular_msg.linear.y =0;
	angular_msg.linear.z =0;
	angular_msg.angular.x = 0;
	angular_msg.angular.y = 0;
	angular_msg.angular.z = 0;

	relative_rotate_angle_radius = relative_rotate_angle * M_PI /180.0;
	ROS_INFO("%f", relative_rotate_angle_radius);
	// set the direction of rotation
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
		ROS_INFO("%f", rotate_angle);
	}while(rotate_angle<relative_rotate_angle_radius);

	// after reach to desired angle, set angle to 0
	angular_msg.angular.z =0;
	velocity_publisher.publish(angular_msg);

}
