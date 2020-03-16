#include "ros/ros.h"
#include <sstream>
#include <cstdlib>
#include <sensor_msgs/LaserScan.h>
#include <iostream>


void scanCallback(const sensor_msgs::LaserScan::ConstPtr & scan_data);

ros::Subscriber scan;
double angle_min = 0.0 ;
double angle_max = 0.0;
float thredhold = 0.022;
std::vector<float> laser_data;



int main(int argc, char **argv)
{
  ros::init(argc, argv, "landmark");
  ros::NodeHandle n;

  scan =  n.subscribe("/scan",1,scanCallback);

  // std::vector<int>  vec{1,2,3,4,5,6,7,8};
  //
  // int vec_size =vec.size();
  // std::cout<<"vec_size!!!!!!!!"<<vec_size<< std::endl;
  // vec[0].resize(1);

  // std::vector <std::vector<int>> matrix(7);
  //
	// for(size_t i=0;i<matrix.size();++i){
	// 	matrix[i].resize(i+1);
	// 	for(size_t j=0;j<matrix[i].size();++j){
	// 		matrix[i][j]=(i+1)*(j+1);
	// 	}
	// }
  //
	// for(size_t i=0;i<matrix.size();++i){
	// 	std::cout<< "matrix["<<i<<"]=={";
	// 	for(size_t j=0;j<matrix[i].size();++j){
	// 		if(j!=0) {
	// 			std::cout<<',';
	// 		}
	// 		std::cout<<matrix[i][j];
	// 	}
	// 	std::cout<<"}\n";
	// }
  // ros::Rate loop_rate(0.1);
  while(ros::ok())
  {
    // std::vector <std::vector<float>> group;
    // int group_size = 0;
    // int count = 0;
    // int laser_data_size = laser_data.size();
    // int start_value = 0;
    // int end_value = 0;
    // //
    // for(int i=0; i < laser_data_size; i++) {
    //   if (std::abs(laser_data[i]-laser_data[i+1])<thredhold) {
    //      // group[group_size][count] = laser_data[i];
    //      count ++;
    //   } else {
    //       if(count > 3) {
    //         // group_size++;
    //         count = 0;
    //         end_value = i;
    //         std::cout<< "size!!!!!!!!!!! "<<end_value-start_value<<std::endl;
    //         // group[group_size].resize(end_value-start_value+1);
    //         group[group_size].resize(360);
    //         // group[i].resize(count);
    //         // int group_size_count = 0;
    //         // for(int j=start_value; j < end_value; j++) {
    //         //   group[group_size][group_size_count] = laser_data[group_size_count];
    //         // }
    //         group_size++;
    //         start_value = i;
    //       } else {
    //       count = 0;
    //       }
    //     }
    // }


    ros::spinOnce();
    ros::Duration(1.0).sleep();
    // loop_rate.sleep();
  }
  return 0;
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr & scan_data){
  laser_data= scan_data->ranges;

  std::vector <std::vector<float>> group(360);
  int group_size = 0; //real group size
  int count = 0;
  int laser_data_size = laser_data.size();
  int start_value = 0; // start_index
  int end_value = 0;   //end_index

  for(int i=0; i < laser_data_size; i++) {
    if (std::abs(laser_data[i]-laser_data[i+1])<thredhold) {
       // group[group_size][count] = laser_data[i];
       count ++;
    } else {
        if(count > 2) {
          // group_size++;
          count = 0;
          end_value = i;
          std::cout<< "size: "<<end_value-start_value<<std::endl;
          group[group_size].resize(end_value-start_value+1);
          int group_size_count = 0; // subgroup;

          for(int j=start_value; j < end_value+1; j++) {
            group[group_size][group_size_count] = laser_data[j];
            group_size_count ++;
          }

          group_size++;
          start_value = i;

        } else {

        count = 0;
        start_value = i;

        }
      }
  }
  // std::cout<< "group: "<<group_size<<std::endl;

  // print value;
  for(int i=0;i<group_size;++i){
    std::cout<< "group["<<i<<"]=={";
    for(size_t j=0;j<group[i].size();++j){
      if(j!=0) {
        std::cout<<',';
      }
      std::cout<<group[i][j];
    }
    std::cout<<"}\n";
  }


  //
  // auto print = [](const double& n) { std::cout << " " << n; };
  // std::cout << "before!!!!!!!!!!!!!!!!!!!!!!!!! :";
  // std::for_each(laser_data.begin(), laser_data.end(), print);

}
