// // odometer: update internal odometry state, publish odometry message ,and broadbast the  odom_frame_id;
// //
// // PUBLISHERS:nav_msgs/Odometry.h(odometry) publish the odometry message
// //
// // SUBSCRIBERS:sensor_msgs/JointState (joint_state): to get wheels velocity

// #include "rigid2d/rigid2d.hpp"
// #include "rigid2d/diff_drive.hpp"
// #include <ros/ros.h>
// #include <sstream>
// #include <geometry_msgs/Twist.h>
// #include <cstdlib>
// #include <sensor_msgs/JointState.h>
// #include <tf/transform_broadcaster.h>
// #include <string>
// #include <nav_msgs/Odometry.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include "nuturtle_robot/start.h"
// #include <cmath>

using std::string;
using rigid2d::WheelVelocities;
using rigid2d::Pose;
using rigid2d::DiffDrive;
using rigid2d::Twist2D;
using rigid2d::PI;

ros::Publisher cmd_vel_publisher;
ros::ServiceServer service;
// int rotation_sign =1;

void publish_velocity(double rotation_velocity);

// bool start_callback(nuturtle_robot::start::Request  &req,
//                   nuturtle_robot::start::Response &resp)
// {
//   if (req.clockwise_forward == 1){
//     rotation_sign = 1;
//   }else {
//     rotation_sign = -1;
//   }
//   return 1;
// }




int main(int argc, char **argv)
{
//   ros::init(argc, argv, "rotation");
//   ros::NodeHandle n;
//   cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

//   double frac_vel = 0.8;
//   double maximum_rotational_velocity_robot = 2.84;
//   n.getParam("frac_vel",frac_vel);
//   n.getParam("maximum_rotational_velocity_robot", maximum_rotational_velocity_robot);
//   ros::ServiceServer service = n.advertiseService("/start", start_callback);

//   double rotation_velocity = frac_vel*maximum_rotational_velocity_robot;
//   ROS_INFO("rotation_velocity:%f", rotation_velocity);
//   double rotation_time = 2.0*PI/rotation_velocity;
//   ROS_INFO("rotation_time:%f", rotation_time);
//   double pub_intervel = 0.02;
//   double loop_number_double = rotation_time / pub_intervel;
//   ROS_INFO("loop_number_double%f",loop_number_double);
//   int loop_number_int = round(loop_number_double);
//   ROS_INFO("loop_number_int%d",loop_number_int);

//   ros::Time start_time,end_time;
  // double duration;

//   ros::Rate rate(1/pub_intervel);
//   ros::Rate rate2(0.1);
//   rate2.sleep();

//   for (int j =0; j<20; j++)
//   {
//     ROS_INFO("rotate!!!ya");
//     for(int i = 0; i<loop_number_int;i++)
//     {
//       publish_velocity(rotation_sign*rotation_velocity);
//         // ROS_INFO("after_publish");
//       rate.sleep();

//     }

//     for(int i = 0; i<loop_number_int/20;i++)
//     {
//         publish_velocity(0);
//         // ROS_INFO("stop");
//         rate.sleep();
//     }
//   }
  ros::spinOnce();

}



void publish_velocity(double rotation_velocity){
  geometry_msgs::Twist rotation_angle;
  rotation_angle.linear.x = 0;
  rotation_angle.linear.x = 0;
  rotation_angle.linear.x = 0;

  rotation_angle.angular.x = 0;
  rotation_angle.angular.y = 0;
  rotation_angle.angular.z = rotation_velocity;
  // ROS_INFO ("%f",rotation_angle.angular.z);

  cmd_vel_publisher.publish(rotation_angle);
}
