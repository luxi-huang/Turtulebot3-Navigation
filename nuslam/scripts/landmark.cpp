#include "ros/ros.h"
#include <sstream>
#include <cstdlib>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include "cmath"
#include <Eigen/Dense>
#include "nuslam/TurtleMap.h"

using rigid2d::Pose;
using rigid2d::Vector2D;
using rigid2d::normalize_angle;
using rigid2d::rad2deg;
using namespace Eigen;

ros::Subscriber scan;
ros::Subscriber odomSub;
Pose pose_odom;
std::vector<std::vector<Vector2D>> position(360);
std::vector<int> circle_group;
std::vector<double> circle_R;
std::vector<double> circle_center_x;
std::vector<double> circle_center_y;
ros::Publisher landmark_pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr & scan_data);
void clustering_groups();
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void check_circle();
double vector_norm(Vector2D vec);
void circle_fitting_algorithm();


double angle_min = 0.0 ;
double angle_max = 0.0;
float thredhold = 0.1;
std::vector<float> laser_data;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "landmark");
  ros::NodeHandle n;

  scan =  n.subscribe("/scan",1,scanCallback);
  odomSub  = n.subscribe<nav_msgs::Odometry>("/odom", 10, odomCallback);
  ros::Publisher landmark_pub = n.advertise<nuslam::TurtleMap>("landmarks", 1);
   //NOTE: add publish mesage

  while(ros::ok())
  {
    ros::spinOnce();
    clustering_groups();
    circle_group.clear();
    check_circle();
    circle_fitting_algorithm();
    ros::Duration(5.0).sleep();
  }
  return 0;
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr & scan_data){
  laser_data.clear();
  laser_data= scan_data->ranges;
  // int size = laser_data.size();
  //
  // auto print = [](const double& n) { std::cout << " " << n; };
  // std::cout << "before!!!!!!!!!!!!!!!!!!!!!!!!! :";
  // std::for_each(laser_data.begin(), laser_data.end(), print);

}



void clustering_groups()
{
  position.clear();
  int group_size = 0; //real group size
  int count = 0;
  int laser_data_size = laser_data.size();
  int start_value = 0; // start_index
  int end_value = 0;   //end_index
  double angle = 0;
  double x_position = 0;
  double y_position = 0;

  for(int i=0; i < laser_data_size; i++) {
    if (std::abs(laser_data[i]-laser_data[i+1])<thredhold) {
       count ++;
    } else {
        if(count > 2) {
          count = 0;
          end_value = i;
          position.resize(group_size+1);
          position[group_size].resize(end_value-start_value+1);
          int group_size_count = 0; // subgroup;
          for(int j=start_value; j < end_value+1; j++) {
            angle = normalize_angle(pose_odom.theta + j);
            x_position = pose_odom.x + laser_data[j]*cos(angle);
            y_position = pose_odom.y + laser_data[j]*sin(angle);

            // angle = normalize_angle(0 + j);
            // x_position = 0 + laser_data[j]*cos(angle);
            // y_position = 0 + laser_data[j]*sin(angle);

            position[group_size][group_size_count].x = x_position;
            position[group_size][group_size_count].y = y_position;
            group_size_count ++;
          }

          group_size++;
          start_value = i+1;

        } else {

        count = 0;
        start_value = i;

        }
      }
  }
}


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	pose_odom.x = msg->pose.pose.position.x;
	pose_odom.y = msg->pose.pose.position.y;

	tf::Quaternion q(
		 msg->pose.pose.orientation.x,
		 msg->pose.pose.orientation.y,
		 msg->pose.pose.orientation.z,
		 msg->pose.pose.orientation.w );

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	pose_odom.theta = yaw;
	// pub_pose_.publish(pose2d);
}


void check_circle(){
  int group_number = position.size();
  // std::vector <std::vector<float>> (360);
  Vector2D point_start;
  Vector2D point_end;
  Vector2D pp;
  Vector2D vec_a;
  Vector2D vec_b;
  double dot_product;
  double norm_a;
  double norm_b;
  double mean_rad = 0;
  double mean_degree = 0;
  double sum_angle = 0;
  double angle = 0;
  double st_deviation = 0;
  double standard_deviation = 0;
  for (int i = 0; i<group_number; i++){
    int group_size = position[i].size();
    ROS_INFO("group_size!!! : %d", group_size);
    point_start = position[i][0];
    point_end = position[i][group_size-1];
    // calculate mean;
    sum_angle = 0;
    for(int j = 1; j<group_size-1; j++){
      pp = position[i][j];
      vec_a.x = point_start.x - pp.x;
      vec_a.y = point_start.y - pp.y;
      vec_b.x = point_end.x - pp.x;
      vec_b.y = point_end.y - pp.y;

      dot_product = (vec_a.x*vec_b.x) + (vec_a.y*vec_b.y);
      norm_a = vector_norm(vec_a);
      norm_b = vector_norm(vec_b);
      sum_angle += acos(dot_product/(norm_a * norm_b));

      // std::vector <std::vector<float>> group(360);
    }


    mean_rad = sum_angle / (group_size-2);
    mean_degree = rad2deg(mean_rad);


    if (mean_degree > 90.0 && mean_degree < 135.0){ //check if mean satisfied the condition
      st_deviation = 0;
      for(int j = 1; j<group_size-1; j++){
        pp = position[i][j];
        vec_a.x = point_start.x - pp.x;
        vec_a.y = point_start.y - pp.y;
        vec_b.x = point_end.x - pp.x;
        vec_b.y = point_end.y - pp.y;
        dot_product = (vec_a.x*vec_b.x)+(vec_a.y*vec_b.y);
        norm_a = vector_norm(vec_a);
        norm_b = vector_norm(vec_b);
        angle = acos(dot_product/(norm_a * norm_b));
        st_deviation += pow(angle-mean_rad,2.0);
        // std::vector <std::vector<float>> group(360);
      }
      standard_deviation = sqrt(st_deviation/(group_size-2));
      if (standard_deviation < 0.15){
        ROS_INFO("st_deviation: %f",standard_deviation);
        circle_group.push_back(i);
      }

    }


  }
  // position[group_size][group_size_count].x

}

double vector_norm(Vector2D vec){
  double norm = 0;
  norm = sqrt(pow(vec.x,2.0)+pow(vec.y,2.0));
  return norm;
}


void circle_fitting_algorithm(){
  int size = 0;
  int group_size = 0; //how many points in a group;
  double sum_x = 0;
  double sum_y = 0;
  double x_hat = 0;
  double y_hat = 0;
  double xx = 0;
  double yy = 0;
  double zz = 0;
  // std::vector<std::vector<double>> circle_x;
  // std::vector<std::vector<double>> circle_y;
  // std::vector<std::vector<double>> circle_z;
  // circle_x.clear();
  // circle_y.clear();
  // circle_z.clear();
  double sum_zz = 0;
  double z_bar = 0;
  double singular_min = 0;
  double compare_value = pow(10,-12.0);
  MatrixXd A_matrix(4, 1);
  size = circle_group.size();
  int eigen_value_size;
  double eigenvalues_min = 0;
  double aa = 0;
  double bb = 0;
  double radius = 0;
  double center_xx = 0;
  double center_yy = 0;

  ROS_INFO("size:%d",size);
  for(int i =0; i < size; i++){
    group_size = position[circle_group[i]].size();
    sum_x = 0;
    sum_y = 0;
    // step two;
    // ROS_INFO("step_two");
    // ROS_INFO("group_size %d",group_size);
    for (int j = 0; j<group_size; j++){
      sum_x += position[circle_group[i]][j].x;
      sum_y += position[circle_group[i]][j].y;
    }
    x_hat = sum_x / group_size;
    y_hat = sum_y / group_size;
    // step three;
    // circle_x[i].resize(group_size);
    // circle_y[i].resize(group_size);
    // circle_z[i].resize(group_size);

    sum_zz = 0;

    MatrixXd Z_matrix(group_size, 4);
    for (int j = 0; j<group_size; j++){
      xx = position[circle_group[i]][j].x - x_hat;
      yy = position[circle_group[i]][j].y - y_hat;
      // circle_x[i][j] = xx;
      // circle_y[i][j] = yy;
      //step four;
      // ROS_INFO("step_four");
      zz = pow(xx,2.0)+pow(yy,2.0);
      sum_zz += zz;
      // circle_z[i][j] = zz;

      //step six;
      // ROS_INFO("step_six");
      Z_matrix(j,0) = zz;
      Z_matrix(j,1) = xx;
      Z_matrix(j,2) = yy;
      Z_matrix(j,3) = 1.0;
     }
    //step five;
    // ROS_INFO("step_five");
    z_bar = sum_zz / group_size;
    // step seven;
    // ROS_INFO("step_seven");
    MatrixXd M_matrix(4, 4);
    M_matrix = (Z_matrix.transpose())*Z_matrix/group_size;
    //step eight, nigh;
    // ROS_INFO("step_eight_nigh");
    MatrixXd H_Matrix_inverse(4,4);
    H_Matrix_inverse << 0.0,0.0,0.0,0.5,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.5,0.0,0.0,-2*z_bar;
    //step ten;
    // ROS_INFO("step_ten");
    JacobiSVD<MatrixXd> svd(Z_matrix, ComputeThinU | ComputeThinV);
    int singluar_size = svd.singularValues().size();
    //step eleven;
    // ROS_INFO("step_eleven");
    singular_min = svd.singularValues()(0);
    for(int m = 0; m < singluar_size; m++){
      if (svd.singularValues()(m) < singular_min){
        singular_min = svd.singularValues()(m);
      }
    }

    if (singular_min < compare_value){
      A_matrix(0,0) = svd.matrixV()(0,3);
      A_matrix(1,0) = svd.matrixV()(1,3);
      A_matrix(2,0) = svd.matrixV()(2,3);
      A_matrix(3,0) = svd.matrixV()(3,3);
    } else {
      //step 12;
      // ROS_INFO("step_twelve");
      MatrixXd Y_matrix;
      Y_matrix = svd.matrixV()*(svd.matrixU().inverse()*Z_matrix);
      MatrixXd Q_matrix;
      Q_matrix = Y_matrix * H_Matrix_inverse * Y_matrix;
      SelfAdjointEigenSolver<MatrixXd> es(Q_matrix);
      MatrixXd A_star_matrix;
      eigen_value_size = es.eigenvalues().size();
      int eigen_count = 0;
      for (int n = 0; n<eigen_value_size; n++){
        ROS_INFO("es.eigenvalues()(n), %f",es.eigenvalues()(n));
        if (es.eigenvalues()(n) > 0 ){
          eigenvalues_min = es.eigenvalues()(n);
          eigen_count = n;
        }
        break;
      }

      for (int n = 0; n<eigen_value_size; n++){
        if (es.eigenvalues()(n) > 0 && es.eigenvalues()(n) < eigenvalues_min){
          eigenvalues_min = es.eigenvalues()(n);
          eigen_count = n;
        }
      }

      ROS_INFO("eigen_count,%d",eigen_count);
      A_star_matrix = es.eigenvectors().col(eigen_count);
      // MatrixXd A_matrix;
      A_matrix = Y_matrix.inverse() *A_star_matrix;
    }

    // step 13;
    // ROS_INFO("step_threen");

    aa = -A_matrix(1,0)/(2*A_matrix(0,0));
    // ROS_INFO("11111");
    bb = -A_matrix(2,0)/(2*A_matrix(0,0));
    // ROS_INFO("2222");
    radius = sqrt((pow(A_matrix(1,0),2.0)+pow(A_matrix(2,0),2.0) - 4*A_matrix(0,0)*A_matrix(3,0))/(4*pow(A_matrix(0,0),2.0)));
    //step 14;
    // ROS_INFO("3333");
    // ROS_INFO("step_fourteen");
    center_xx = aa + x_hat;
    center_yy = bb + y_hat;
    // ROS_INFO("4444");
    ROS_INFO("radius: %f", radius);
    // for (size_t m = 0; m<circle_R, m++ )
    circle_R.push_back(radius);
    circle_center_x.push_back(center_xx);
    circle_center_y.push_back(center_yy);

  }
}
