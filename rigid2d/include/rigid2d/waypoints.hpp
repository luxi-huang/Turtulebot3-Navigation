#ifndef waypoints_INCLUDE_GUARD_HPP
#define waypoints_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include <iosfwd> // contains forward definitions for iostream objects
#include "cmath"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <vector>


namespace rigid2d{
  struct Velocity{
    double linear = 0;
    double angular = 0;
  };

  class Waypoints{
    // std::vector<int> v1 = {1,2};
    std::vector<Vector2D> point1={{0.5,0.0}};
    std::vector<Vector2D> point2={{1.0,0.5}};
    std::vector<Vector2D> point3={{0.5,1.0}};
    std::vector<Vector2D> point4={{0.0,0.5}};
    std::vector<Velocity> vel = {{0,0}};
    std::vector<Twist2D> tw = {{0,0,0}};
    Vector2D goal;

  public:
    //set the goal
    Waypoints(Vector2D G1,Vector2D G2,Vector2D G3,Vector2D G4);

    // calcualte the rest_distance to goal point;
    double left_distance(Pose pose);

    // calculate left_angle;
    double left_angle(Pose pose);

    // calculate next twist velocities;
    void velocity_array(double rest_distance,double rest_angle, double steps);

    // change_goal;
    void change_goal();

    // set condition to move to nextWaypoint;
    void nextWaypoint(double rest_distance,double rest_angle,double threshold_linear);


    void convert_velocity_to_twist();

    // use twist array to update pose;
    void update_current_pose(DiffDrive & a);


  };
}
#endif
