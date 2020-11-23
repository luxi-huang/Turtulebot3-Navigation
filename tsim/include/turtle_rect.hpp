#ifndef TURTLE_RECT_HPP
#define TURTLE_RECT_HPP

//*****add library******//
#include "ros/ros.h"
#include <sstream>
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "std_srvs/Empty.h"
#include <cstdlib>
#include <cmath>
#include "tsim/PoseError.h"

namespace turtle_rect
{
    class TurtleRect 
    {
    private:
        //***************** NODE HANDLES ***************//
        ros::NodeHandle nh_;

        // //***************** NODE SERVICE ***************//
        // ros::ServiceClient client;
        
        //***************** NODE PUBLISHER ***************//
        ros::Publisher vel_pub;
        ros::Publisher PoseError_pub;
        //***************** NODE PUBLISHER ***************//
        ros::Subscriber pose_subs;
        //******************** PARAMETERS *****************// 
        /* data from yaml file */  
        double x;
        double y;
        double width;
        double height;
        double trans_vel;
        double rot_vel;
        double frequency;
        /* other parameters */
        turtlesim::Pose turtlesim_pose; 
        turtlesim::Pose predict_pose;
         
        /* state machine flag */
        // int state;
        // int path_state;


    public:
        //***************** FUNCTIONS **********************//
        TurtleRect(ros::NodeHandle* nodehandle);
        void getting_parameter();
        int turtle_setpen_client(int off);
        int Teleport_client();
        void turtle_move(int state);
        void initial_publishers_subscribers();
        void pose_callback(const turtlesim::Pose::ConstPtr & pose_message);
        void go_stright(double speed, double distance);
        void rotate(double speed, double angle);
        void predict(float dist,float rot);
        void init_predict_pose();
        void Error_pose();
    };

}    

#endif



