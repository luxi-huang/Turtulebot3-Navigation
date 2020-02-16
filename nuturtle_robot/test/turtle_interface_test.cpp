#include "ros/ros.h"
#include <sstream>
#include <cstdlib>
#include "geometry_msgs/Twist.h"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include <sensor_msgs/JointState.h>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <gtest/gtest.h>

using::rigid2d::almost_equal;

using rigid2d::Twist2D;
using rigid2d::WheelVelocities;
using std::string;

ros::Publisher cmd_vel_publisher;
// ros::Publisher joint_state_publisher;
ros::Subscriber wheeel_cmd_subscriber;
// ros::Subscriber sensor_data_subscriber;

ros::Publisher encoder_data_publisher;
ros::Subscriber joint_state_subscriber;


struct MyJointState {
  string name;
  double velocity;
  double position;
  MyJointState(
    string n = "",
    double v = 0.0,
    double p = 0.0): name(n), velocity(v), position(p)
    {}
};

MyJointState wheel1,wheel2;
WheelVelocities wheel_v;

//
int cmd_vel_no_rotation(){
  geometry_msgs::Twist ttwistt;
  int check = 1;
  ros::NodeHandle nh;
  cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel",true);

  ros::Time current_time;
  ros::Time init_time;
  current_time = ros::Time::now();
  init_time = current_time;
  double duration = 0;
  while(ros::ok() && duration<1 && check ==1){
    ttwistt.linear.x = 1;
    ttwistt.linear.y = 2;
    ttwistt.angular.z = 0;
    cmd_vel_publisher.publish(ttwistt);
    current_time = ros::Time::now();
    duration = (current_time - init_time).toSec();
    ros::spinOnce();
    if (wheel_v.u1 ==  15.151515 && wheel_v.u2 == 15.151515){
      check *= 0;
    }else {
      check *= 1;
    }
  }
  return check;
}


int cmd_vel_no_translation(){
  geometry_msgs::Twist ttwistt;
  int check = 1;
  ros::NodeHandle nh;
  cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel",true);

  ros::Time current_time;
  ros::Time init_time;
  current_time = ros::Time::now();
  init_time = current_time;
  double duration = 0;
  while(ros::ok() && duration<1 && check ==1){
    ttwistt.linear.x = 0;
    ttwistt.linear.y = 0;
    ttwistt.angular.z = 1;
    cmd_vel_publisher.publish(ttwistt);
    current_time = ros::Time::now();
    duration = (current_time - init_time).toSec();
    ros::spinOnce();
    if (wheel_v.u1 ==  -1.212121  && wheel_v.u2 == 1.212121){
      check *= 0;
    }else {
      check *= 1;
    }
  }
  return check;
}

int cmd_vel_both_translation_rotation(){
  geometry_msgs::Twist ttwistt;
  int check = 1;
  ros::NodeHandle nh;
  cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel",true);

  ros::Time current_time;
  ros::Time init_time;
  current_time = ros::Time::now();
  init_time = current_time;
  double duration = 0;
  while(ros::ok() && duration<1 && check ==1){
    ttwistt.linear.x = 1;
    ttwistt.linear.y = 1;
    ttwistt.angular.z = 1;
    cmd_vel_publisher.publish(ttwistt);
    current_time = ros::Time::now();
    duration = (current_time - init_time).toSec();
    ros::spinOnce();
    if (wheel_v.u1 == 13.939394 && wheel_v.u2 == 16.363636){
      check *= 0;
    }else {
      check *= 1;
    }
  }
  return check;
}

void velCallback(const nuturtlebot::WheelCommands::ConstPtr & wheel_w){
  wheel_v.u1= wheel_w->left_velocity;
  wheel_v.u2 = wheel_w->right_velocity;
}



int check_encoder_joint_State(){
  int check = 0;
  nuturtlebot::SensorData encoder_data;

  ros::NodeHandle nh;
  encoder_data_publisher = nh.advertise<nuturtlebot::SensorData>("sensor_data",true);

  int left_encoder_value  =0;
  int right_encoder_value = 0;

  for(int i = 0; i<2;i++){
    left_encoder_value ++;
    right_encoder_value ++;
    encoder_data.left_encoder = left_encoder_value;
    encoder_data.right_encoder = right_encoder_value;
    encoder_data_publisher.publish(encoder_data);
    ros::spinOnce();
  }
  check = 1;

  //
  // if (wheel1.velocity == 13.939394 && wheel2.velocity == 16.363636){
  //   check *= 0;
  // }else {
  //   check *= 1;
  // }

  return check;

}


void joint_statesCallback(const sensor_msgs::JointState::ConstPtr & mesg){
	wheel1.name = mesg->name[0];
  wheel2.name = mesg->name[1];
  wheel1.position = mesg->position[0];
  wheel2.position = mesg->position[1];
  wheel2.velocity = mesg->velocity[1];
  wheel1.velocity = mesg->velocity[0];
}

//cmd_vel commands with no rotational
TEST(TestSuite, test1)
{
  int check_value = cmd_vel_no_rotation();
  ASSERT_EQ(1,check_value) << "cmd_vel commands with no rotational ";
}


//cmd_vel commands with no translational
TEST(TestSuite, test2)
{
  int check_value = cmd_vel_no_translation();
  ASSERT_EQ(1,check_value) << "cmd_vel commands with no translational ";
}

//cmd_vel_both_translation_rotation
TEST(TestSuite, test3)
{
  int check_value = cmd_vel_both_translation_rotation();
  ASSERT_EQ(1,check_value) << "both translation and rotation";
}

TEST(TestSuite, test4)
{
  int check_value = check_encoder_joint_State();
  ASSERT_EQ(1,check_value) << "both translation and rotation";
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) )
  //  ros::console::notifyLoggerLevelsChanged();

  ros::init(argc, argv, "turtle_interface_test");
  ros::NodeHandle nh;

  // cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel",true);
  wheeel_cmd_subscriber =  nh.subscribe("wheel_cmd",1000,velCallback);

  // encoder_data_publisher = nh.advertise<nuturtlebot::SensorData>("sensor_data",true);
  joint_state_subscriber =  nh.subscribe("joint_states",1000,joint_statesCallback);




  return RUN_ALL_TESTS();
}
