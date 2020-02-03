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

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
int turtle_setpen_client(int off);
int traj_rest_client();
int Error_pose(int x, int y, int theta);
void go_to_goal(turtlesim::Pose  goal_pose, double distance_tolerance);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_rect");
	ros::NodeHandle n;
	publich: n;
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",60);
	PoseError_publisher = n.advertise<tsim::PoseError>("pose_error", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
  std::vector<double> waypoints_x =  {3,7,9,5,1};
  std::vector<double> waypoints_y =  {2,3,7,10,6};
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

	// ros::Rate loop_rate(1000);
  int value = 0;
	while (ros::ok()){
		// ros::spinOnce();
		ros::Duration(2).sleep();
		traj_rest_client();
		ros::Duration(2).sleep();
		turtle_setpen_client(0);


    while (1){
      ros::spinOnce();
      if (value == 4){
        value =0;
      } else{
        value += 1;
      }



      turtlesim::Pose  goal_pose;

      goal_pose.x = waypoints_x[value];
      goal_pose.y = waypoints_y[value];

      double distance_tolerance = 0.001;
      go_to_goal(goal_pose, distance_tolerance);
    }
	}
}




void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
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
		// goal_msg.angular.z =4*(atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x-turtlesim_pose.x)-turtlesim_pose.theta);
		double current_angle = atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x-turtlesim_pose.x)-turtlesim_pose.theta;
		while (current_angle < -M_PI){
			current_angle = current_angle +2*M_PI;
		}
		ROS_INFO ("%f\n",current_angle);
		goal_msg.angular.z = 4*current_angle;
		// goal_msg.angular.z = angle;z

		velocity_publisher.publish(goal_msg);

		ros::spinOnce();
		loop_rate.sleep();
		left_distance = sqrt(pow((turtlesim_pose.x-goal_pose.x),2)+pow((turtlesim_pose.y-goal_pose.y),2));
	}while(left_distance>distance_tolerance);
	goal_msg.linear.x =0;
	goal_msg.angular.z = 0;
	velocity_publisher.publish(goal_msg);
}
