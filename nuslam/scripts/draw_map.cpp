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

#include "nuslam/TurtleMap.h"

#include <functional>  // To use std::bind
#include <visualization_msgs/Marker.h>

// GLOBAL VARS
nuslam::TurtleMap global_map;
ros::Subscriber landmark_sub;

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



int main(int argc, char **argv)
{
  ros::init(argc, argv, "draw_map");
  ros::NodeHandle n;

  // scan =  n.subscribe("/scan",1,scanCallback);
  // odomSub  = n.subscribe<nav_msgs::Odometry>("/odom", 10, odomCallback);
  // ros::Publisher landmark_pub = n.advertise<nuslam::TurtleMap>("landmarks", 1);
	// marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  landmark_sub = n.subscribe("/landmarks", 1, mapCallback);


  while(ros::ok())
  {
    ros::spinOnce();

  }
  return 0;
}


void mapCallback(const nuslam::TurtleMap::ConstPtr&map)
{
  int size = map->radi.size();
  circle_R.resize(size);
  circle_center_x.resize(size);
  circle_center_y.resize(size);

  for(int i = 0; i< size; i++ ){
    circle_R[i] = map->radi[i];
    circle_center_x[i] = map->x_pose[i];
    circle_center_y[i] = map->y_pose[i];
  }
}
