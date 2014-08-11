///////////////////////////////////////////////////////////////////////////////
//Source for i90_movement node to send movement commands to i90							 //
//v1.0 																																			 //
//First creation 																														 //
//Huseyin Emre Erdem 																												 //
//08.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////

/*This node reads the target position information published from the i90_sensor node
under i90_target topic. The type of message published is std_msgs::Float32MultiArray. 
After acquiring the target, it turns and goes to the target.*/

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "Float32.h"
#include "Float32MultiArray.h"

/*Prototypes*/
void targetCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

/*Main function*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "i90_movement");//Create node called "i90_movement"
	ros::NodeHandle n;//Create nodehandler to modify features of the node
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("drrobot_cmd_vel", 1);
	ros::Subscriber sub = n.subscribe("i90_ir", 1, irCallback);
	ros::Subscriber sub = n.subscribe("i90_sonar", 1, sonarCallback);

	ros::Duration d = ros::Duration(2,0);
	ros::Rate loop_rate(2);
	geometry_msgs::Twist cmdvel_;//commands to be sent to i90

	while (ros::ok())
	{
		cmdvel_.linear.x = 0.1;
		ROS_INFO("Speed: %f", cmdvel_.linear.x);
		pub.publish(cmdvel_);
		// d.sleep();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void targetCallback(const std_msgs::Float32MultiArray::ConstPtr& targetValues){
	ROS_INFO("Target values: [%f]/t[%f]", targetValues.data[0], targetValues.data[1]);
}

