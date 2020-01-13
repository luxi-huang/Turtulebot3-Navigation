#include "ros/ros.h"
#include <sstream>
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

using namespace std; // use for count

ros::Publisher velocity_publisher;
// ros::Subscriber pose_subscriber;
// turtlesim::Pose turtlesim_pose;

void go_stright(double speed, double distance, bool ifForward);
// void rotate (double angular_speed, double angle, bool clockwise);
// double degrees2radians(double angle_in_degrees);
// void setDesiredOrientation (double desired_angle_radians);
// void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
// void moveGoal(turtlesim::Pose  goal_pose, double distance_tolerance);
// void gridClean();
// void spiralClean();

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_rect");
	ros::NodeHandle n;

	double speed, angular_speed;
	double distance, angle;
	bool ifForward, clockwise;


	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
  go_stright(2.0, 9.0, true);

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
