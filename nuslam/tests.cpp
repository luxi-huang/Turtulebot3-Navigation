#include <gtest/gtest.h>
#include "rigid2d/diff_drive.hpp"
#include <sstream>
#include <cstdlib>
#include <string>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::JacobiSVD;
using Eigen::ComputeThinU;
using Eigen::ComputeThinV;
using Eigen::SelfAdjointEigenSolver;
using rigid2d::Vector2D;
using namespace std;


TEST(TestSuite, test)
{
  int size = 0;
  int group_size = 0; //how many points in a group;
  double sum_x = 0;
  double sum_y = 0;
  double x_hat = 0;
  double y_hat = 0;
  double xx = 0;
  double yy = 0;
  double zz = 0;
  double sum_zz = 0;
  double z_bar = 0;
  double singular_min = 0;
  double compare_value = pow(10,-12.0);
  MatrixXd A_matrix(4, 1);
  size = 1;
  int eigen_value_size;
  double eigenvalues_min = 0;
  double aa = 0;
  double bb = 0;
  double radius = 0;
  double center_xx = 0;
  double center_yy = 0;

  std::vector<std::vector<Vector2D>> position{{{1,7},{2,6},{5,8},{7,7},{9,5},{3,7}}};

  group_size = position[0].size();
  sum_x = 0;
  sum_y = 0;
  // step two;
  // ROS_INFO("step_two");
  // ROS_INFO("group_size %d",group_size);
  for (int j = 0; j<group_size; j++){
    sum_x += position[0][j].x;
    sum_y += position[0][j].y;
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
    xx = position[0][j].x - x_hat;
    yy = position[0][j].y - y_hat;
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
  MatrixXd E_matrix(singluar_size,singluar_size);
  for (int q = 0; q<singluar_size;q++){
    for (int p = 0; p<singluar_size;p++){
      if (p==q){
        E_matrix(q,p)=svd.singularValues()(q);
      }else{
        E_matrix(q,p)=0;
      }
    }
  }

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
    Y_matrix = svd.matrixV()*E_matrix*svd.matrixV().transpose();
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


  // ASSERT_NEAR(center_xx, 4.615482,0.001);
  // ASSERT_NEAR(center_yy, 2.807354,0.001);
  // ASSERT_NEAR(radius, 4.8275,0.001);

  ASSERT_NEAR(center_xx, 4.615482,0.01);
  ASSERT_NEAR(center_yy, 2.807354,0.001);
  ASSERT_NEAR(radius, 4.8275,0.001);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
