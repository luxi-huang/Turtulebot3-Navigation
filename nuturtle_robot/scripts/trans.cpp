/// \file
/// \brief: This rotation node is to send out rotation velocities when the robot rotates.
///
/// PUBLISHERS:turtle1/cmd_vel (geometry_msgs/Twist): publish twist command on cmd_vel
///
/// SERVICES: start (nuturtle_robot/Start): service to start the rotation

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "nuturtle_robot/start.h"
#include <cmath>

using namespace std; 

class Rotation
{
public: 
    Rotation(ros::NodeHandle &nh) 
    {
      nh.getParam("frac_vel",frac_vel);
      nh.getParam("maximum_translational_velocity", max_trans_vel);

      service = nh.advertiseService("/start", &Rotation::start_callback, this);
      cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
      
      double rotation_velocity = frac_vel * max_trans_vel;
      //   ROS_INFO("rotation_velocity:%f", rotation_velocity);

      double translation_time = 0.2 / rotation_velocity;
      double pub_intervel = 0.005;
      double loop_number_double = translation_time / pub_intervel;
      int loop_number_int = round(loop_number_double);

      ros::Rate rate(1/pub_intervel);
      
      ros::Rate rate2(0.1);
      rate2.sleep();

      for (int j = 0; j < 10; j++)
      {
        ROS_INFO("trans!!!ya");
        for(int i = 0; i < loop_number_int; i++)
        {
          publish_velocity(rotation_sign*rotation_velocity);
          rate.sleep();
        }

        for(int i = 0; i < loop_number_int / 10;i++)
        {
          publish_velocity(0);
          rate.sleep();
        }
      }

      ros::spinOnce();
    }  

    bool start_callback(nuturtle_robot::start::Request  &req,
                  nuturtle_robot::start::Response &resp)
    {
      if (req.clockwise_forward == 1) {
        rotation_sign = 1;
      } else {
        rotation_sign = -1;
      }
  
      return 1;
    }

  void publish_velocity(double translation_velocity){
    geometry_msgs::Twist translation;
    translation.linear.x = translation_velocity;
    translation.linear.y = 0;
    translation.linear.z = 0;

    translation.angular.x = 0;
    translation.angular.y = 0;
    translation.angular.z = 0;
    // ROS_INFO ("%f",rotation_angle.angular.z);
    cmd_vel_publisher.publish(translation);
  }


private: 
  double max_rot_vel_;
  ros::Publisher cmd_vel_publisher;
  ros::ServiceServer service;
  int rotation_sign =1;
  double frac_vel;
  double max_trans_vel;
  ros::Time start_time,end_time;


};


int main(int argc, char **argv)
{
    // init the node
    ros::init(argc, argv, "rotation");
    ros::NodeHandle nh;
    Rotation my_rotation = Rotation(nh);

    ros::spin();
    return 0;
}