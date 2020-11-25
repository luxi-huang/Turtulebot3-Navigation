/* ******************************************************************************************************* 
 * FILE DESCRIPTIONS                                                                                      
 *  
 * NODE: 
 *  + turtle_rect ～ enable the turtle to follow the trajectory of a rectangle.
 *  
 * PUBLISHERS: 
 *  + turtle1/cmd_vel (Twist) ~ publish turtle velocity.
 *  + pose_error (Pose_error) ~ publish the pose error of turtle.
 * 
 * SUBSCRIBERS:
 *  + turtle1/pose (Pose) ～ subscribe the current pose of the turtle1 turtle (x, y, theta)
 * 
 * SERVICE:
 *  + traj_reset (std_srvs::Empty): Reset the trajectory, take turtle back to left corner of rectangle. 
 **********************************************************************************************************/


#include "turtle_rect.hpp"

using namespace turtle_rect;
int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "turtle_rect");
    ros::NodeHandle n;
    TurtleRect DoorClass(&n);
}
