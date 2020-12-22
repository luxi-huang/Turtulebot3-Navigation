# Package: Rigid2d

Author: Luxi Huang

## Package summary:
This package is an implementation of 2D robot transform and differential Drive Kinematics based on  [Modern Robotics](http://hades.mech.northwestern.edu/index.php/Modern_Robotics) by Kevin Lynch. 

## Files:

### rigid2d.hpp/cpp 
- This library performing 2D rigid body transformations, and it also did vector, and twist Operation.  

### Diff_drive.hpp/cpp
- This library will track the state of a differential drive robot as its wheel positions as updated.  

### Waypoints.hpp/cpp
- This library helps to calculate sequence of forward and angular velocities requires for a robot to travel between waypoints. 

### scripts/Odometry.cpp 
- This file create a node that publish Odometry messages in a standard ROS way.

### scripts/fake_diff_encoders.cpp
- This file creates fake encoders to control robot on simulation.










