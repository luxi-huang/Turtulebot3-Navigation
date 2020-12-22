/*******************************************************
 * Author: Luxi Huang
 * This class will track the state of a differential drive robot as its 
 * wheel positions as updated.
 * *********************************************************/

#ifndef diff_drive_INCLUDE_GUARD_HPP
#define diff_drive_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include<iosfwd> // contains forward definitions for iostream objects
#include "cmath"
#include "rigid2d/rigid2d.hpp"

namespace rigid2d
{
  // std::ostream & operator<<(std::ostream & os, const Pose & pose);
  // std::istream & operator>>(std::istream & is, Pose & pose);

  struct WheelVelocities
  {
    double ul = 0.0;
    double ur = 0.0;
  };

  std::ostream & operator<<(std::ostream & os, const  WheelVelocities & wheel_v);
  std::istream & operator>>(std::istream & is, WheelVelocities & wheel_v);

  class DiffDrive
  {
  private:
    Transform2D pose_;
    double whe_base;
    double whe_radius;
    // double L = 5.0;

  public:
      /// \brief the default constructor creates a robot at (0,0,0), with a fixed wheel base and wheel radius
      DiffDrive();

      ///  \brief create a DiffDrive model by specifying the pose, and geometry
      /// \param pose - the current position of the robot
      /// \param wheel_base - the distance between the wheel centers
      /// \param wheel_radius - the raidus of the wheels
      explicit DiffDrive(Transform2D pose,double wheel_base, double wheel_radius);

      /// \brief determine the wheel velocities required to make the robot
      ///        move with the desired linear and angular velocities
      /// \param twist - the desired twist in the body frame of the robot
      /// \returns - the wheel velocities to use
      /// \throws std::exception
      WheelVelocities twistToWheels(Twist2D t);
      
      /// \brief determine the body twist of the robot from its wheel velocities
      /// \param vel - the velocities of the wheels, assumed to be held constant
      ///  for one time unit
      /// \returns twist in the original body frame of the
      Twist2D wheelsToTwist(WheelVelocities vel);
      //
      /// \brief Update the robot's odometry based on the current encoder readings
      /// \param left - the left encoder angle (in radians)
      /// \param right - the right encoder angle (in radians)
      void updateOdometry(double left_radians, double right_radians);
      
      /// \brief update the odometry of the diff drive robot, assuming that
      /// it follows the given body twist for one time  unit
      /// \param cmd - the twist command to send to the robot
      void feedforward(Twist2D cmd);
      //
      /// \brief get the current pose of the robot
      Transform2D getpose();
      //
      /// \brief get the wheel speeds, based on the last encoder update
      /// \returns the velocity of the wheels, which is equivalent to
      /// displacement because \Delta T = 1
      WheelVelocities wheelVelocitie(double delta_radians_L, double delta_radians_R) const;
      //
      /// \brief reset the robot to the given position/orientation
      void reset(rigid2d::Twist2D ps);

  };

}

#endif
