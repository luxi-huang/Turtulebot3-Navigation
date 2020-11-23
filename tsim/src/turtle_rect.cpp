#include "turtle_rect.hpp"

namespace turtle_rect
{
	TurtleRect::TurtleRect(ros::NodeHandle* nodehandle):nh_(*nodehandle)
	{
		/* THIS IS CLASS CONSTRCTOR */
		getting_parameter();
		initial_publishers_subscribers();
		turtle_move(0);
		// ros::spin();

	}

	void TurtleRect::getting_parameter() 
	{
		/* GET PARAMETERS FROM YAML FILE */
		nh_.getParam("x", x); // The x coordinate of the lower left corner of a rectangle
		nh_.getParam("y",y); // The y coordinate of the lower left corner of a rectangle
		nh_.getParam("width",width); // The width of the rectangle
		nh_.getParam("height",height); // The height of the rectangles
		nh_.getParam("trans_vel",trans_vel); // The translational velocity of the robot
		nh_.getParam("rot_vel",rot_vel); // The rotational velocity of the robot
		nh_.getParam("frequency",frequency); // The frequency of the control loop
		std::cout << "x" << x << "\n";
	}

	void TurtleRect::initial_publishers_subscribers()
	{	
		/* subscribers */
		pose_subs = nh_.subscribe("/turtle1/pose", 1, &TurtleRect::pose_callback,this);
		/* publishers */
		vel_pub = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
		PoseError_pub= nh_.advertise<tsim::PoseError>("pose_error", 1000);
	}

	void TurtleRect::pose_callback(const turtlesim::Pose::ConstPtr & pose_message) {
		/* This is turtle pose subscrber callback function */ 
		turtlesim_pose.x=pose_message->x;
		turtlesim_pose.y=pose_message->y;
		turtlesim_pose.theta=pose_message->theta;
	}

	int TurtleRect::turtle_setpen_client(int off) 
	{	
		/*  Setup set pen survice;
		*	 Lift pen when input is true; */
		ros::ServiceClient client= nh_.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
		turtlesim::SetPen SetPen_srv;
		SetPen_srv.request.r = 0;
		SetPen_srv.request.g = 0;
		SetPen_srv.request.b = 0;
		SetPen_srv.request.width = 1;
		SetPen_srv.request.off = off;

		ros::service::waitForService("turtle1/set_pen");
		if (client.call(SetPen_srv))
			{
				ROS_INFO("Pen Lifted");
			}
			else
			{
				ROS_ERROR("Failed to call service set_pen");
				return 1;
			}
		return 0;
	}

	int TurtleRect::Teleport_client() 
	{
		/*  Setup set pen survice;
		*	teleport turtle to the left corner of window; */
		ros::ServiceClient client= nh_.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
		turtlesim::TeleportAbsolute teleport_absolute_req;
		teleport_absolute_req.request.x = x;
		teleport_absolute_req.request.y = y;
		teleport_absolute_req.request.theta = 0;

		ros::service::waitForService("turtle1/teleport_absolute");
		if (client.call(teleport_absolute_req))
			{
				ROS_INFO("Teleport to left corner");
			}
			else
			{
				ROS_ERROR("Failed to call service teleport_absolute");
				return 1;
			}
		return 0;
	}

	void TurtleRect::go_stright(double speed, double distance)
	{
		/*This function control robot go straight,
		* and calculate pose error. */

		// setup speed;
		geometry_msgs::Twist go_stright_msg;
		go_stright_msg.linear.x = speed;
		go_stright_msg.linear.y = 0;
		go_stright_msg.linear.z = 0;
		go_stright_msg.angular.x = 0;
		go_stright_msg.angular.y = 0;
		go_stright_msg.angular.z = 0;

		ros::Rate loop_rate(frequency);
		int times = (int) (distance / speed * frequency);
		float dist = 0.0;
		float rot = 0.0;

		for (int i = 0; i < times; i++)
		{
			vel_pub.publish(go_stright_msg);
			dist +=  distance / (float) times;
			predict(dist,rot);
			Error_pose();
			ros::spinOnce();
			loop_rate.sleep();
		}
		// after arrived,set the velocity equal to immediately
		go_stright_msg.linear.x = 0;
		vel_pub.publish(go_stright_msg);
	}


	void TurtleRect::rotate(double speed, double angle)
	{
		// setup speed;
		geometry_msgs::Twist rotate_msg;
		rotate_msg.linear.x = speed;
		rotate_msg.linear.y = 0;
		rotate_msg.linear.z = 0;
		rotate_msg.angular.x = 0;
		rotate_msg.angular.y = 0;
		rotate_msg.angular.z = speed;

		ros::Rate loop_rate(frequency);
		int times = (int) (angle / speed * frequency);
		float dist = 0.0;
		float rot = 0.0;

		for (int i = 0; i < times; i++)
		{
			vel_pub.publish(rotate_msg);
			rot +=  speed / (float) times;
			predict(dist,rot);
			Error_pose();
			ros::spinOnce();
			loop_rate.sleep();
		}

		// after arrived,set the velocity equal to immediately
		rotate_msg.angular.z = 0;
		vel_pub.publish(rotate_msg);
	}

	void TurtleRect::predict (float dist,float rot){
			predict_pose.theta += rot;
			predict_pose.x += dist*cos(predict_pose.theta);
			predict_pose.y += dist*sin(predict_pose.theta);
	}

	void TurtleRect::Error_pose(){
		tsim::PoseError p_error;
		p_error.x_error = turtlesim_pose.x - predict_pose.x;
		p_error.y_error = turtlesim_pose.y - predict_pose.y;
		p_error.theta_error = turtlesim_pose.theta - predict_pose.theta;
		PoseError_pub.publish(p_error);
	}


	void TurtleRect::init_predict_pose(){
		predict_pose.x = 0.0;
		predict_pose.y = 0.0;
		predict_pose.theta = 0.0;
	}	

	void TurtleRect::turtle_move(int state) 
	{
		/* Setup robot state machine */
		switch (state)
		{
		// case 0: inital the robot	
		case 0:
			/*  1. lift pen
			*	2. teleport robot 
			*	3. put pen down */
			turtle_setpen_client(1); 
			Teleport_client();
			turtle_setpen_client(0);  
			init_predict_pose();
			state = 1;
			break;
		// case 1: move forward;
		case 1:
			go_stright(trans_vel, x);
			rotate (rot_vel, 90 * M_PI/180.0);
		// case 2:

			

		default:
			// compiling error 
			ROS_INFO("Unexpected State.");
			break;
		}
	} 




}	




