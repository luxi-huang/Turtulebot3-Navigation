#include "turtle_rect.hpp"

using namespace turtle_rect;
int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "turtle_rect");
    ros::NodeHandle n;
    TurtleRect DoorClass(&n);
}
