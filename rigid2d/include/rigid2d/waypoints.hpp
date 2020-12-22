/*************************************************************
 * Author: Luxi Huang
 * This class helpes to control the robot trajectory in waypoints
 * by sending its forward and angular velocities 
 * **********************************************************/

#ifndef waypoints_INCLUDE_GUARD_HPP
#define waypoints_INCLUDE_GUARD_HPP

#include <iosfwd> // contains forward definitions for iostream objects
#include "cmath"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <vector>


namespace rigid2d {
  
  enum CurrentState
  {
    Rotate_left, // robot state: rotate to left 
    Rotate_right, // robot state: rotate to right 
    Trans  //  robot state: go straight  
  };
  
  class Waypoints 
  {
  private: 
    
    std::vector<Vector2D> points;
    // std::vector<Velocity> vel;
    Twist2D static_vel;
    // std::vector<Velocity> static_vel;
    int current_goal;
    DiffDrive myDiffDrive;
    double frequency; 
    double linearThreshold;
    double angularThreshold;
    CurrentState state_;
    
  public:
    /// 
    /// \brief: setup function with default private vairables; 
    Waypoints();

    /// \brief: initial private vairables
    /// \param: diff_drive;
    /// \param: waypoints;
    /// \param: static_vel;
    /// \param: threshold for linear and angle;
    /// \param: frequency;
    explicit Waypoints(std::vector<Vector2D> waypoints, Twist2D vel, DiffDrive my_diff, int hz, int l_thred, int a_thred);

    /// \brief: the pipline of combining all functions and moving forward
    void pipeline();

    /// \brief: update diff_drive state by feed forward 
    /// \param: cmd 
    void update_pose(Twist2D cmd);

    /// \brief:update state base on the current position and goal point 
    void update_state();

    /// \brief: generate next point velocity;
    /// \returns a Twist2D command
    Twist2D nextWaypoint();

    /// \brief: control robot to move forward:
    Twist2D move_forward_cmd();
    
    /// \brief: control robot to turn left; 
    Twist2D rotate_left();

    /// \brief: control robot to turn left; 
    Twist2D rotate_right();

    /// \brief: update robot next goal;
    void update_goal();
    
    bool ifClose(double x1, double y1, double x2, double y2); 
  };
}
#endif
