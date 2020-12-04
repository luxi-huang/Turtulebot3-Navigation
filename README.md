# README for TURTLEBOT3 NAVIGATION PROJECT 

Author: Luxi Huang

## Project Overview:
This totally-from-scratch project involved modeling a Turtlebot3 in Gazebo and using Differential Drive Kinematics to perform Odometry calculations. The Gazebo Plugin was developed to emulate the low-level interface on the real Turtlebot3 for the ability to develop high-fidelity code in simulation. I also implemented landmark detection on the Turtlebot3’s LIDAR, and used these features to perform EKF SLAM with Unknown Data Association.


The core project components are:

- rigid2d library containing 2D Lie Group operations for Transforms, Vectors and Twists as well as differential drive robot kinematics for odometry updates.
- nuturtle_description: houses the description of a differential drive robot with a caster wheel for support.
- nuturtle_robot: interfaces with the real Turtlebot3’s low-level controls such as setting wheel speeds and reading sensors.
- nuturtle_gazebo: contains a Gazebo Plugin to emulate the Turtlebot3’s low-level controls in Gazebo for the ability to develop in simulation.
- nuslam: library containing LIDAR feature detection methods and EKF SLAM implementation with Unknown Data Association.


##  HOW to install files:
### Option 1: 
1. Create a workspace 
   
  > $ mkdir -p ~/catkin_ws/src 

  > $ cd ~/catkin_ws/src

2. Download .rosinstall file to ```~catkin_ws\src``` directory
3. Use ```wstool``` to install all packages on ```.rosinstall``` file
    >  wstool update
4. > catkin_make

  
### Option 2: 
1. create catkin_ws 
  > $ mkdir -p ~/catkin_ws/src 

  > $ cd ~/catkin_ws/src
2. clone all packages 
   >  git clone https://github.com/luxi-huang/Turtulebot3-Navigation.git
   
   >  git clone https://github.com/ME495-Navigation/nuturtlebot.git
3.  > catkin_make 