/// PUBLISHES:
///   scan/marker (visualization_msgs::Marker): publishes markers to indicate detected landmark positions
///
/// SUBSCRIBES:
///   /landmarks_node/landmarks (nuslam::TurtleMap), stores lists of x,y coordinates and radii of detected landmarks
///
/// FUNCTIONS:
///   mapCallback (void): callback for /landmarks_node/landmarks subscriber, which stores TurtleMap data for exraction

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>
#include <string>
#include <vector>
#include <boost/iterator/zip_iterator.hpp>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"
#include "nuslam/TurtleMap.h"

#include <functional>  // To use std::bind
#include <visualization_msgs/Marker.h>

// GLOBAL VARS
nuslam::TurtleMap global_map;
ros::Subscriber landmark_sub;
ros::Publisher marker_pub;

using rigid2d::Pose;
// void mapCallback(const nuslam::TurtleMap &map)
// {
//   /// \brief store TurtleMap data for converting into marker coordinates
//   /// \param nuslam::TurtleMap, which stores lists of x,y coordinates and radii of detected landmarks
//   global_map = map;
//
//   // std::cout << "NUM OF CLUSTERS: " << global_map.radii.size() << std::endl;
//
//   callback_flag = true;
// }

// struct MyLandmark {
//   double xx;
//   double yy;
//   double radius;
//   MyLandmark(
//     double x = 0.0,
//     double y = 0.0,
//     double r = 0.0): xx(x), yy(y), radius(r)
//     {}
// };
std::vector<double> circle_R;
std::vector<double> circle_center_x;
std::vector<double> circle_center_y;

//
// MyLandmark landM;



void mapCallback(const nuslam::TurtleMap::ConstPtr&map);
void publish_marker(Pose p_pose,int number,double radi);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "draw_map");
  ros::NodeHandle n;

  // scan =  n.subscribe("/scan",1,scanCallback);
  // odomSub  = n.subscribe<nav_msgs::Odometry>("/odom", 10, odomCallback);
  // ros::Publisher landmark_pub = n.advertise<nuslam::TurtleMap>("landmarks", 1);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  landmark_sub = n.subscribe("/landmarks", 1, mapCallback);

  while(ros::ok())
  {
    ros::spinOnce();

    int landmark_size = circle_R.size();
    if (landmark_size >0){
      double value = 0;
      value = circle_R[0];
      if (value != 0){
        for (int i = 0; i<landmark_size;i++){
          Pose P_pose;
          double landmark_radi;
          P_pose.x = circle_center_x[i];
          P_pose.y = circle_center_y[i];
          landmark_radi = circle_R[i];
          publish_marker(P_pose,i,landmark_radi);
        }
      }
    }

  }
  return 0;
}


void mapCallback(const nuslam::TurtleMap::ConstPtr&map)
{
  int size = map->radi.size();
  // circle_R.clear();
  // circle_center_x.clear();
  // circle_center_y.clear();


  circle_R.resize(size);
  circle_center_x.resize(size);
  circle_center_y.resize(size);
  if (map->radi[0] != 0){
    for(int i = 0; i< size; i++ ){
      circle_R[i] = map->radi[i];
      circle_center_x[i] = map->x_pose[i];
      circle_center_y[i] = map->y_pose[i];
    }
  }
}


void publish_marker(Pose p_pose,int number,double radi){
  // visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/odom_frame_id";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = number;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  // marker.type = shape;
  marker.type = visualization_msgs::Marker::CYLINDER;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = p_pose.x;
  marker.pose.position.y = p_pose.y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = radi;
  marker.scale.y = radi;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);

}
