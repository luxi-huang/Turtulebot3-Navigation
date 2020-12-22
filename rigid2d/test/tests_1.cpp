/******************************************************** 
 * Author: Luxi Huang
 * This is a test file for rigid2D and diffdrive library.
 ********************************************************/

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <cstring>
#include <cmath>
#include <fstream>

using namespace std;
using namespace rigid2d;

TEST(TestSuite, almost_equal1)
{
  int a;
  ROS_INFO ("enter 1");
  a = almost_equal(0, 0);
  ASSERT_EQ(1,a) << "is zero faild";
}

TEST(TestSuite, almost_equal2)
{
  int a;
  a = almost_equal(0.001, 0.005, 1.0e-2);
  ASSERT_EQ(1,a)<< "is_zero failed" ;
}

TEST(TestSuite, almost_equal3)
{
  int a;
  a = almost_equal(deg2rad(0.0), 0.0);
  ASSERT_EQ(1,a) << "deg2rad failed";
}

TEST(TestSuite, almost_equal4)
{
  int a;
  a = almost_equal(rad2deg(0.0), 0.0);
  ASSERT_EQ(1,a) << "rad2deg) failed";
}

TEST(TestSuite, almost_equal5)
{
  int a;
  a = almost_equal(deg2rad(rad2deg(2.1)), 2.1);
  ASSERT_EQ(1,a) << "deg2rad failed";
}

// Test identity Transform2D;
TEST(TestSuite, InputOutput)
{
    Transform2D trans_test;
    ostringstream out;

    istringstream in("90 1 4");

    in >> trans_test;
    out << trans_test;

    std::string string_ = out.str();
    const char *str1 = string_.c_str();

    const char *str2 = "degrees:1.5708 dx:1 dy:4 \n";

    ASSERT_STREQ(str1, str2);
}

TEST(TestSuite, normalize_angle)
{
  double a;
  a = normalize_angle(3*PI);
  ASSERT_EQ(PI,a) << "deg2rad failed";
}

TEST(TestSuite, TransferFunction_Vec_Rad)
{
  Vector2D v_ab;
  double degree_ab, radians_ab;
  v_ab.x =1;
  v_ab.y =1;
  degree_ab=30;
  radians_ab = deg2rad(degree_ab);
  Transform2D Tab(v_ab, radians_ab);
  std::stringstream buffer;
  buffer << "Tab : "<<Tab;

  const char *str2 = "Tab : degrees:0.523599 dx:1 dy:1 \n";
  ASSERT_EQ(buffer.str(),str2);
}

// //test operator *=
TEST(TestSuite, Identify)
{
  Vector2D v_ab;
  double degree_ab, radians_ab;
  v_ab.x =1;
  v_ab.y =1;
  degree_ab=30;
  radians_ab = deg2rad(degree_ab);
  Transform2D Tab(v_ab, radians_ab);

  Transform2D I;
  I = Transform2D();

  Tab *= I;
  std::stringstream buffer;
  buffer << "Tab : "<<Tab;

  const char *str2 = "Tab : degrees:0.523599 dx:1 dy:1 \n";
  ASSERT_EQ(buffer.str(),str2);
  
}

// //Test *
TEST(TestSuite, Transfer2DMultiplicate)
{
  Vector2D v_ab, v_bc;
  double degree_ab, radians_ab,  degree_bc, radians_bc;
  v_ab.x =1;
  v_ab.y =1;
  degree_ab=30;
  radians_ab = deg2rad(degree_ab);
  Transform2D Tab(v_ab, radians_ab);

  v_bc.x =2;
  v_bc.y =1;
  degree_bc=60;
  radians_bc = deg2rad(degree_bc);
  Transform2D Tbc(v_bc, radians_bc);
  Transform2D Tac;
  Tac = Tab*Tbc;

  std::stringstream buffer;
  buffer << "Tac : "<<Tac;
  
  const char *str2 = "Tac : degrees:1.5708 dx:2.23205 dy:2.86603 \n";
  ASSERT_EQ(buffer.str(),str2);
}

TEST(TestSuite, Transferwithvector)
{
  Vector2D v_ab, v_bc, v, va;
  double degree_ab, radians_ab,  degree_bc, radians_bc;
  v_ab.x =1;
  v_ab.y =1;
  degree_ab=30;
  radians_ab = deg2rad(degree_ab);
  Transform2D Tab(v_ab, radians_ab);

  v_bc.x =2;
  v_bc.y =1;
  degree_bc=60;
  radians_bc = deg2rad(degree_bc);
  Transform2D Tbc(v_bc, radians_bc);
  Transform2D Tac;
  Tac = Tab*Tbc;

  v.x =1;
  v.y =2;

  va = Tac(v);

  std::stringstream buffer;
  buffer << "va : "<<va.x<<","<<va.y;

   const char *str2 = "va : 0.232051,3.86603";
  ASSERT_EQ(buffer.str(),str2);
}

TEST(TestSuite, vectorplus)
{
  Vector2D v1,v2,v3;
  v1.x = 1;
  v1.y = 2;

  v2.x = 3;
  v2.y = 4;

  v3 = v1+v2;
  std::stringstream buffer;
  buffer << "v :"<<v3.x<<","<<v3.y;
  ASSERT_EQ(buffer.str(),"v :4,6");
}

TEST(TestSuite, vectorplus2)
{
  Vector2D v1,v2;
  v1.x = 1;
  v1.y = 2;

  v2.x = 3;
  v2.y = 4;

  v1+=v2;
  std::stringstream buffer;
  buffer << "v :"<<v1.x<<","<<v1.y;
  ASSERT_EQ(buffer.str(),"v :4,6");
}

TEST(TestSuite, vector_minus)
{
  Vector2D v1,v2,v3;
  v1.x = 1;
  v1.y = 2;

  v2.x = 3;
  v2.y = 4;

  v3 = v1-v2;
  std::stringstream buffer;
  buffer << "v :"<<v3.x<<","<<v3.y;
  ASSERT_EQ(buffer.str(),"v :-2,-2");
}

//test twist -= sign
TEST(TestSuite, vector_minus2)
{
  Vector2D v1,v2,v3;
  v1.x = 1;
  v1.y = 2;

  v2.x = 3;
  v2.y = 4;

  v1-=v2;
  std::stringstream buffer;
  buffer << "v :"<<v1.x<<","<<v1.y;
  ASSERT_EQ(buffer.str(),"v :-2,-2");
}

// //Test operator vector*s
TEST(TestSuite, VectorMultiplier)
{
  Vector2D v1,v2;
  double s=2;

  v1.x = 1;
  v1.y = 2;
  v2 = v1*s;

  std::stringstream buffer;
  buffer << "v :"<<v2.x<<","<<v2.y;
  ASSERT_EQ(buffer.str(),"v :2,4");
}

// Test vector operator scaler*=vector
TEST(TestSuite, vectorMultiplier2)
{
  Vector2D v1,v2;
  double s=2;
  v1.x = 1;
  v1.y = 2;

  v1*=s;
  std::stringstream buffer;
  buffer << "v :"<<v1.x<<","<<v1.y;
  ASSERT_EQ(buffer.str(),"v :2,4");
}

TEST(TestSuite, length)
{
  Vector2D v1;
  v1.x = 1;
  v1.y = 1;

  double len;
  len = v1.length(v1);

  EXPECT_FLOAT_EQ(len,1.4142135);
}

TEST(TestSuite, distance)
{
  Vector2D v1,v2;
  v1.x = 1;
  v1.y = 1;

  v2.x = 2;
  v2.y = 2;

  double distance;
  distance = v1.distance(v1,v2);

  EXPECT_FLOAT_EQ(distance,1.4142135);
}

TEST(TestSuite, VectorAngle)
{
  Vector2D v1;
  v1.x = 1;
  v1.y = 2;

  double angle;
  angle = v1.angle(v1);

  EXPECT_FLOAT_EQ(angle,1.1071488);
}

///  \NOTE:changed format of displacement, need to change agagin 
TEST(Rigid2dTest, Inverse)
{
    Vector2D translation;

    translation.x = 0;
    translation.y = -1;
    double degree, angle;
    degree = 90;
    angle = deg2rad(degree);

    Transform2D trans_test(translation, angle);

    rigid2d::Transform2D result = trans_test.inv();

    // Transform2D newtrans;
    
    // double x, y, theta; 
    // result.displacement(x,y,theta);
    
    std::stringstream buffer;
    buffer << "newtrans : "<<result;
  
    const char *str2 = "newtrans : degrees:-1.5708 dx:1 dy:6.12323e-17 \n";
    ASSERT_EQ(buffer.str(),str2);
}

//the following tests are from diff_drive.cpp
// test twistTowheels works;
TEST(TestSuite, twistToWheels)
{
  Twist2D t;
  WheelVelocities u;

  double whe_base = 2.0;
  double whe_radius = 2.0;
  t.theta_dot = 2.0; //2.0
  t.vx = 1.0;  //1.0
  t.vy = 0.0;  //0.0
  t.theta_dot = 0.0;

  DiffDrive Diff_drive_test;
  u = Diff_drive_test.twistToWheels(t);

  // double wheel_radius = 0.1;

  // std::stringstream buffer;
  // buffer <<"u1: " << u.u1 << " u2: " << u.u2 << " u3: "<< u.u3 << " u4: "<< u.u4;
  ASSERT_EQ(1.0,u.ul);
}

// // test wheeltotwist function;
// TEST(TestSuite, test28){
//   WheelVelocities vel;
//   Twist2D t;
//   double whe_base = 2.0;
//   double D = whe_base/2.0;
//   double whe_radius =2.0;
//   double r = whe_radius/2.0;

//   vel.u1 = -0.5;
//   vel.u2 = 1.5;
//   vel.u3 = 1.5;
//   vel.u3 = -0.5;

//   t.vx = (vel.u1+vel.u2)*r;
//   t.theta_dot = (vel.u2/D-vel.u1/D)*r;
//   t.vy= 0;

//   // t.theta_dot = (vel.u1+vel.u2)*whe_radius/2.0;
//   // t.vx = whe_radius*(vel.u2-vel.u1)/(2.0*D);
//   // t.vy = 0;
//   std::stringstream buffer;
//   buffer <<"theta_dot: " << t.theta_dot << " vx: " << t.vx << " vy: "<< t.vy;
//   ASSERT_EQ(buffer.str(),"theta_dot: 2 vx: 1 vy: 0");
// }

// //test feedforward withonly angle;
// TEST(TestSuite, test29){

//   Twist2D cmd;
//   cmd.theta_dot = 2.0;
//   cmd.vx = 0;
//   cmd.vy = 0;
//   DiffDrive D;
//   D.feedforward(cmd,1);
//   Pose p;
//   p = D.pose();
//   std::stringstream buffer;
//   buffer <<"Pose_theta " << p.theta << " x " << p.x << "y "<< p.y;
//   ASSERT_EQ(buffer.str(),"Pose_theta 2 x 0y 0");
// }

// // test feedforward with only linear velocity;
// TEST(TestSuite, test30){
//   Twist2D cmd;
//   cmd.theta_dot = 0;
//   cmd.vx = 1;
//   cmd.vy = 0;
//   DiffDrive D;
//   D.feedforward(cmd,1);
//   Pose p;
//   p = D.pose();
//   std::stringstream buffer;
//   buffer <<"Pose_theta " << p.theta << " x " << p.x << "y "<< p.y;
//   ASSERT_EQ(buffer.str(),"Pose_theta 0 x 1y 0");
// }

// //test feedforward with only angular velocity;
// TEST(TestSuite, test31){
//   Twist2D cmd;
//   cmd.theta_dot = 1;
//   cmd.vx = 1;
//   cmd.vy = 0;
//   DiffDrive D;
//   D.feedforward(cmd,1);
//   Pose p;
//   p = D.pose();
//   std::stringstream buffer;
//   buffer <<"Pose_theta " << p.theta << " x " << p.x << "y "<< p.y;
//   ASSERT_EQ(buffer.str(),"Pose_theta 1 x 0.540302y 0.841471");
// }

int main(int argc, char **argv){
  ROS_INFO("hello11111");
  testing::InitGoogleTest(&argc, argv);
  ROS_INFO("hello22222");
  // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) )
  //  ros::console::notifyLoggerLevelsChanged();

  ros::init(argc, argv, "tester");
  ROS_INFO("hello333333");
  ros::NodeHandle nh;
  ROS_INFO("hello44444");
  return RUN_ALL_TESTS();
}
