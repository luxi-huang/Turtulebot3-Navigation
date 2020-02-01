#include "rigid2d/rigid2d.hpp"
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>
#include <cmath>
#include <fstream>

// using namespace std;
using namespace rigid2d;

// TEST(TestSuite, test)
// {
//   struct Vector2D v;
//
//   // std::cout << "enter 1 for v.x\n";
//   ROS_INFO ("enter 1 for v.x\n");
//   std::cin >> v.x;
//   // std::cout << "enter 1 for v.y\n";
//   ROS_INFO ("enter 1 for v.y\n");
//   std::cin >> v.y;
//
//   std::stringstream buffer;
//   buffer << "v :"<<v.x<<","<<v.y;
//   ASSERT_EQ(buffer.str(),"v :1,1");
// }



TEST(TestSuite, test1)
{
  int a;
  ROS_INFO ("enter 1");
  a = almost_equal(0, 0);
  ASSERT_EQ(1,a) << "is zero faild";
}

TEST(TestSuite, test2)
{
  int a;
  a = almost_equal(0.001, 0.005, 1.0e-2);
  ASSERT_EQ(1,a)<< "is_zero failed" ;
}

TEST(TestSuite, test3)
{
  int a;
  a = almost_equal(deg2rad(0.0), 0.0);
  ASSERT_EQ(1,a) << "deg2rad failed";
}

TEST(TestSuite, test4)
{
  int a;
  a = almost_equal(rad2deg(0.0), 0.0);
  ASSERT_EQ(1,a) << "rad2deg) failed";
}


TEST(TestSuite, test5)
{
  int a;
  a = almost_equal(deg2rad(rad2deg(2.1)), 2.1);
  ASSERT_EQ(1,a) << "deg2rad failed";
}

// Test identity Transform2D;
TEST(TestSuite, test6)
{
  Transform2D I;
  I = Transform2D();

  std::stringstream buffer;
  buffer << "I : "<<I;
  ASSERT_EQ(buffer.str(),"I : degree0 dx 0dy 0");
}

//Test Transform2D Tab(v_ab);
TEST(TestSuite, test7)
{
  struct Vector2D v_ab;
  v_ab.x =1;
  v_ab.y =1;
  Transform2D Tab(v_ab);

  std::stringstream buffer;
  buffer << "Tab : "<<Tab;
  ASSERT_EQ(buffer.str(),"Tab : degree0 dx 1dy 1");
}

// test Transform2D Tab(radians_ab)
TEST(TestSuite, test8)
{
  struct Vector2D v_ab;
  double degree_ab, radians_ab;
  v_ab.x =1;
  v_ab.y =1;
  degree_ab=30;
  radians_ab = deg2rad(degree_ab);
  Transform2D Tab(radians_ab);
  std::stringstream buffer;
  buffer << "Tab : "<<Tab;
  ASSERT_EQ(buffer.str(),"Tab : degree30 dx 0dy 0");
}

//test Transform2D Tab(v_ab, radians_ab);
TEST(TestSuite, test9)
{
  struct Vector2D v_ab;
  double degree_ab, radians_ab;
  v_ab.x =1;
  v_ab.y =1;
  degree_ab=30;
  radians_ab = deg2rad(degree_ab);
  Transform2D Tab(v_ab, radians_ab);

  std::stringstream buffer;
  buffer << "Tab : "<<Tab;
  ASSERT_EQ(buffer.str(),"Tab : degree30 dx 1dy 1");
}


// test inv;
TEST(TestSuite, test10)
{
  struct Vector2D v_ab;
  double degree_ab, radians_ab;
  v_ab.x =1;
  v_ab.y =1;
  degree_ab=30;
  radians_ab = deg2rad(degree_ab);
  Transform2D Tab(v_ab, radians_ab);
  Transform2D Tba;

  Tba = Tab.inv();
  std::stringstream buffer;
  buffer << "Tba : "<<Tba;
  ASSERT_EQ(buffer.str(),"Tba : degree-30 dx -1.36603dy -0.366025");
}

//test operator *=
TEST(TestSuite, test11)
{
  struct Vector2D v_ab;
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
  ASSERT_EQ(buffer.str(),"Tab : degree30 dx 1dy 1");
}

//Test *
TEST(TestSuite, test12)
{
  struct Vector2D v_ab, v_bc;
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
  ASSERT_EQ(buffer.str(),"Tac : degree90 dx 2.23205dy 2.86603");
}

// test operatpr ()
TEST(TestSuite, test13)
{
  struct Vector2D v_ab, v_bc, v, va;
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
  ASSERT_EQ(buffer.str(),"va : 0.232051,3.09808");
}



// TEST(TestSuite, test14)
// {
//   struct Vector2D v;
//
//   // std::cout << "enter 1 for v.x\n";
//   ROS_INFO ("enter 1 for v.x");
//   std::cin >> v.x;
//   // std::cout << "enter 1 for v.y\n";
//   ROS_INFO ("enter 1 for v.y\n");
//   std::cin >> v.y;
//
//   std::stringstream buffer;
//   buffer << "v :"<<v.x<<","<<v.y;
//   ASSERT_EQ(buffer.str(),"v :1,1");
// }

// integrate twist test;
TEST(TestSuite, test15)
{
  Twist2D tw1;
  Vector2D v;
  tw1.vx = 1;
  tw1.vy = 2;
  v = intergrateTwist(tw1);

  std::stringstream buffer;
  buffer << "v :"<<v.x<<","<<v.y;
  ASSERT_EQ(buffer.str(),"v :1,2");
}

TEST(TestSuite, test16)
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

TEST(TestSuite, test17)
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

TEST(TestSuite, test18)
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
TEST(TestSuite, test19)
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

//Test operator vector*s
TEST(TestSuite, test20)
{
  Vector2D v1,v2;
  v1.x = 1;
  v1.y = 2;

  double s=2;


  v2 = v1*s;
  std::stringstream buffer;
  buffer << "v :"<<v2.x<<","<<v2.y;
  ASSERT_EQ(buffer.str(),"v :2,4");
}

// test operator s*vector
TEST(TestSuite, test21)
{
  Vector2D v1,v2;
  v1.x = 1;
  v1.y = 2;

  double s=2;


  v2 = s*v1;
  std::stringstream buffer;
  buffer << "v :"<<v2.x<<","<<v2.y;
  ASSERT_EQ(buffer.str(),"v :2,4");
}

// Test vector operator vector*=scaler
TEST(TestSuite, test22)
{
  Vector2D v1,v2;
  v1.x = 1;
  v1.y = 2;

  double s=2;


  v1*=s;
  std::stringstream buffer;
  buffer << "v :"<<v1.x<<","<<v1.y;
  ASSERT_EQ(buffer.str(),"v :2,4");
}


// Test vector operator scaler*=vector
TEST(TestSuite, test23)
{
  Vector2D v1,v2;
  v1.x = 1;
  v1.y = 2;

  double s=2;


  s*=v1;
  std::stringstream buffer;
  buffer << "v :"<<v1.x<<","<<v1.y;
  ASSERT_EQ(buffer.str(),"v :2,4");
}


// Test operator length
TEST(TestSuite, test24)
{
  Vector2D v1;
  v1.x = 1;
  v1.y = 1;

  double len;
  len = v1.length(v1);

  EXPECT_FLOAT_EQ(len,1.4142135);
}

TEST(TestSuite, test25)
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


TEST(TestSuite, test26)
{
  Vector2D v1;
  v1.x = 1;
  v1.y = 2;



  double angle;
  angle = v1.angle(v1);

  EXPECT_FLOAT_EQ(angle,1.1071488);
}


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
