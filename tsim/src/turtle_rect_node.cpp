#include "turtle_rect.hpp"

using namespace std; 
using namespace turtle_rect;
int x;
int y;
int width;
int height;
int trans_vel;
int rot_vel;
int frequency;


int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "turtle_rect");
    ros::NodeHandle n;
    TurtleRect DoorClass(&n);
}
