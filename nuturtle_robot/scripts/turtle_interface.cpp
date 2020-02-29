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

using rigid2d::Twist2D;
using rigid2d::DiffDrive;
using rigid2d::Pose;
using rigid2d::WheelVelocities;
using rigid2d::PI;

ros::Publisher wheel_cmd_publisher;
ros::Publisher joint_state_publisher;
ros::Subscriber cmd_vel_subscriber;
ros::Subscriber sensor_data_subscriber;

void velCallback(const geometry_msgs::Twist::ConstPtr & Twist);
void sensor_Callback(const nuturtlebot::SensorData::ConstPtr & sensor);
void pub_wheel_velocity(WheelVelocities v);
void publish_joint_state(WheelVelocities v);



WheelVelocities wheel_sensor;
Twist2D ttwist_value;
int new_left =0, new_right =0;
int last_left, last_right;
int maximum_rotational_velocity_motor;
int first_value = 0;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_interface");
  ros::NodeHandle n;

  cmd_vel_subscriber =  n.subscribe("turtle1/cmd_vel",1,velCallback);
  wheel_cmd_publisher = n.advertise<nuturtlebot::WheelCommands>("wheel_cmd", true);

  sensor_data_subscriber = n.subscribe("sensor_data",1,sensor_Callback);
  joint_state_publisher = n.advertise<sensor_msgs::JointState>("joint_states",true);


  double wheel_base;
  double wheel_radius;
  double encoder_ticks;

  n.getParam("wheel_base",wheel_base);
  n.getParam("wheel_radius",wheel_radius);
  n.getParam("encoder_ticks",encoder_ticks);
  n.getParam("maximum_rotational_velocity_motor", maximum_rotational_velocity_motor);

  DiffDrive diff1;
  Pose pp;
  pp.x = 0;
  pp.y = 0;
  pp.theta = 0;


  Pose P;

  diff1 = DiffDrive(pp,wheel_base,wheel_radius);

  last_right = new_right;
  last_left = new_left;

  double duration;
  ros::Time current_time;
  ros::Time last_time;
  current_time = ros::Time::now();
  last_time = current_time;

  double left_wheel_velocity,right_wheel_velocity = 0;
  ros::Rate loop_rate(10);
  ROS_INFO("wheelBase: %f ", wheel_base);
  ROS_INFO("wheel_radius: %f", wheel_radius);
  while(ros::ok()){

    WheelVelocities wheel_v;
    wheel_v = diff1.twistToWheels(ttwist_value);

    pub_wheel_velocity(wheel_v);

    current_time = ros::Time::now();
    duration = (current_time - last_time).toSec();
    last_time = current_time;

    // left_wheel_velocity = ((last_left - new_left)/duration/encoder_ticks)*2*PI;
    // right_wheel_velocity = ((last_right - new_right)/duration/encoder_ticks)*2*PI;

    left_wheel_velocity = ((new_left-last_left)/duration/encoder_ticks)*2*PI;
    right_wheel_velocity = ((new_right - last_right)/duration/encoder_ticks)*2*PI;

    last_right = new_right;
    last_left = new_left;


    WheelVelocities wheel_v_encoder;
    wheel_v_encoder.u1 = left_wheel_velocity;
    wheel_v_encoder.u2 = right_wheel_velocity;

    publish_joint_state(wheel_v_encoder);

    ros::spinOnce();
    loop_rate.sleep();
  }
}



void velCallback(const geometry_msgs::Twist::ConstPtr & Twist){
  ttwist_value.vx = Twist->linear.x;
  ttwist_value.vy = Twist->linear.y;
  ttwist_value.theta_dot=Twist->angular.z ;
}


void pub_wheel_velocity(WheelVelocities v){
  nuturtlebot::WheelCommands v_cmd;
  // ROS_INFO("v1 %f", v.u1);

  if (v.u1 > maximum_rotational_velocity_motor){
    v.u1 = maximum_rotational_velocity_motor;
  }
  if (v.u2 > maximum_rotational_velocity_motor){
    v.u2 = maximum_rotational_velocity_motor;
  }
  if (v.u1 < -maximum_rotational_velocity_motor){
    v.u1 = -maximum_rotational_velocity_motor;
  }
  if (v.u2 < -maximum_rotational_velocity_motor){
    v.u2 = -maximum_rotational_velocity_motor;
  }

  // ROS_INFO("v1 %f", v.u1);
  v_cmd.left_velocity = (v.u1/maximum_rotational_velocity_motor)*265;
  // ROS_INFO("v/max_v %f", v.u1/maximum_rotational_velocity_motor);
  v_cmd.right_velocity = (v.u2/maximum_rotational_velocity_motor)*265;
  // ROS_INFO ("%f", v.u1);
  wheel_cmd_publisher.publish(v_cmd);
}

void sensor_Callback(const nuturtlebot::SensorData::ConstPtr & sensor){
  new_left = sensor->left_encoder;
  new_right = sensor->right_encoder;

  if (first_value == 0){
    last_left = new_left;
    last_right = new_right;
    first_value = 1;
  }
  // ROS_INFO("wheel_encoder left:!!!!!!! %d ", new_left);
  // ROS_INFO("wheel_encoder right:!!!!!!!!! %d ", new_right);
}

void publish_joint_state(WheelVelocities v){
  std::vector<std::string> joint_name_vector;
  sensor_msgs::JointState j_s;
  joint_name_vector.push_back("odometer/left_wheel_joint");
  joint_name_vector.push_back("odometer/right_wheel_joint");

  j_s.name.resize(joint_name_vector.size());
  j_s.position.resize(joint_name_vector.size());
  j_s.velocity.resize(joint_name_vector.size());

  j_s.name[0] = joint_name_vector[0];
  j_s.name[1] = joint_name_vector[1];
  j_s.header.stamp = ros::Time::now();
  j_s.velocity[0] = v.u1;
  j_s.velocity[1] = v.u2;
  // ROS_INFO("j_s.velocity_v1 %f", j_s.velocity[0]);
  // ROS_INFO("j_s.velocity_v2 %f", j_s.velocity[1]);
  joint_state_publisher.publish(j_s);
}
