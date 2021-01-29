/// \file
/// \brief Node of the turtle interface:  low-level control and odometry routines
///
/// PUBLISHES:
///     joint_state (sensor_msgs/JointState): publish joint states based on sensor data
///     wheel_cmd (nuturtlebot/WheelCommands): publish wheel velocity to move the robot
/// SUBSCRIBES:
///     turtle1/cmd_vel (geometry_msgs/Twist): subscribe to current robot cmd_vel commands
///     sensor_data (nuturtlebot/SensorData): subscribe sensor (encoder) data

#include "ros/ros.h"
#include <sstream>
#include <cstdlib>
#include "geometry_msgs/Twist.h"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
// #include "nuturtlebot/SensorData.h"
#include <sensor_msgs/JointState.h>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <string>

using namespace std;
class TurtleInterface
{
public: 
  TurtleInterface(ros::NodeHandle &nh)
  {
    // initial subscriber and publisher
    cmd_vel_subscriber =  nh.subscribe("turtle1/cmd_vel",1,&TurtleInterface::velCallback, this);
    wheel_cmd_publisher = nh.advertise<nuturtlebot::WheelCommands>("wheel_cmd", true);

    sensor_data_subscriber = nh.subscribe("sensor_data",1,&TurtleInterface::sensor_Callback, this);
    joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states",true);
  
    nh.getParam("wheel_base",wheel_base);
    nh.getParam("wheel_radius",wheel_radius);
    nh.getParam("encoder_ticks",encoder_ticks);
    nh.getParam("maximum_translational_velocity", max_trans_vel);
    nh.getParam("maximum_rotational_velocity_robot", max_rot_vel);
    nh.getParam("maximum_rotational_velocity_motor", max_moter_vel);
    nh.setParam("/left_wheel_joint", "left_wheel_joint");
    nh.setParam("/right_wheel_joint", "right_wheel_joint");
    nh.getParam("/left_wheel_joint", left_wheel_joint);
    nh.getParam("/right_wheel_joint",right_wheel_joint);

    //initial robot
    // initial robot;
    rigid2d::Transform2D initPose;
    myRobot = rigid2d::DiffDrive(initPose, wheel_base, wheel_radius);
  }

  void velCallback(const geometry_msgs::Twist::ConstPtr & Twist) 
  {
    Twist_value.vx = Twist->linear.x;
    Twist_value.vy = Twist->linear.y;
    Twist_value.theta_dot=Twist->angular.z;

    // check if velocity excess boundary
    if (Twist_value.vx > max_trans_vel)
    {
      Twist_value.vx = max_trans_vel;
    }

    if (Twist_value.theta_dot > max_rot_vel)
    {
      Twist_value.theta_dot = max_rot_vel;
    }

    //convert twist to wheelvel 
    rigid2d::WheelVelocities wheel_v;
    wheel_v = myRobot.twistToWheels(Twist_value);

    //

    if (wheel_v.ul > max_moter_vel){
      wheel_v.ul = max_moter_vel;
    }
    if (wheel_v.ur > max_moter_vel){
      wheel_v.ur = max_moter_vel;
    }
    if (wheel_v.ul < -max_moter_vel){
      wheel_v.ul = -max_moter_vel;
    }
    if (wheel_v.ur < -max_moter_vel){
      wheel_v.ur = -max_moter_vel;
    }

    nuturtlebot::WheelCommands wheel_cmd_msg;

    wheel_cmd_msg.left_velocity = std::round(wheel_v.ul/max_moter_vel)*265;
    wheel_cmd_msg.right_velocity = std::round(wheel_v.ur/max_moter_vel)*265;
    wheel_cmd_publisher.publish(wheel_cmd_msg);
  }

  void sensor_Callback(const nuturtlebot::SensorData::ConstPtr & sensor) {
    double left_pos, right_pos;
    left_pos = (sensor->left_encoder / encoder_ticks) * 2 * rigid2d::PI;
    right_pos = (sensor->right_encoder / encoder_ticks) * 2 * rigid2d::PI;
    
    sensor_msgs::JointState j_s;
    j_s.header.stamp = ros::Time::now();

    std::vector<std::string> joint_name_vector;
    joint_name_vector.push_back(left_wheel_joint);
    joint_name_vector.push_back(right_wheel_joint);
    j_s.name = joint_name_vector;

    std::vector<double> position;
    position.push_back(right_pos);
    position.push_back(left_pos);
    j_s.position = position;
    
    joint_state_publisher.publish(j_s);
  }  


private: 
  ros::Publisher wheel_cmd_publisher;
  ros::Publisher joint_state_publisher;
  ros::Subscriber cmd_vel_subscriber;
  ros::Subscriber sensor_data_subscriber;
  rigid2d::DiffDrive myRobot;
  

  double wheel_base;
  double wheel_radius;
  double encoder_ticks;
  double max_moter_vel;
  double max_trans_vel;
  double max_rot_vel;

  rigid2d::Twist2D Twist_value;
  string left_wheel_joint;
  string right_wheel_joint;
};  

int main(int argc, char **argv)
{
  // init the node
  ros::init(argc, argv, "turtle_interface");
  ros::NodeHandle nh;
  TurtleInterface my_turtle_interface = TurtleInterface(nh);

  ros::spin();
  return 0;
}