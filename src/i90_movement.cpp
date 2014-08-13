///////////////////////////////////////////////////////////////////////////////
//Source for i90_movement node to send movement commands to i90							 //
//v1.0 																																			 //
//First creation 																														 //
//Huseyin Emre Erdem 																												//
//Edited by:																																//
//Edgar Buchanan Berumen																										 //
//12.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////

/*This node reads the target position information published from the i90_sensor node
under i90_target topic. The type of message published is std_msgs::Float32MultiArray. 
After acquiring the target, it turns and goes to the target.*/

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "Float32.h"
#include "Float32MultiArray.h"

/*Libraries*/

#include <math.h>

/*Prototypes*/
void move_to_target_pos(const std_msgs::Float32MultiArray::ConstPtr& msg);
void get_current_pos(const std_msgs::Float32MultiArray::ConstPtr& msg);

/*Variables*/

float current_pos_x;
float current_pos_y;
float target_pos_x;
float target_pos_y;
float distance;
float slope;
float angle;
int turn_pulses;
int distance_pulses;
int current_pulses_1; //Ask some node to retrieve encoders information
int current_pulses_2; //Ask some node to retrieve encoders information
int target_pulses_1;
int target_pulses_2;

/*Main function*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "i90_movement");//Create node called "i90_movement"
	ros::NodeHandle n;//Create nodehandler to modify features of the node
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("drrobot_cmd_vel", 1);
	ros::Subscriber sub = n.subscribe("i90_target_pos", 1, move_to_target_pos);
	ros::Subscriber sub = n.subscribe("i90_current_pos", 1, get_current_pos);
	ros::Publisher pub = n.advertise<std_msgs::Empty>("i90_translation_done",1);
	ros::Publisher pub = n.advertise<std_msgs::Empty>("i90_rotation_done",1);

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

void move_target_pos(const std_msgs::Float32MultiArray::ConstPtr& targetValues){
	/*Turn i90*/
	slope = (target_pos_y-current_pos_y)/(target_pos_x-current_pos_x); //Calculate slope between target and current positions
	//angle = atan (slope) * 180 / 3.1416;	
	angle = atan2 (target_pos_y-current_pos_y, target_pos_x-current_pos_x) * 180 / 3.1416; //Calculate angle in degrees between target and current positions
	turn_pulses = int (angle / 0.235); //Calculate quantity of encoder's pulses to reach angle.
	target_pulses_1 = current_pulses_1 + turn_pulses;
	//Calculate target pulses	
	if(target_pulses_1 >= 32768) target_pulses_1 = target_pulses_1 - 32768;
	target_pulses_2 = current_pulses_2 - turn_pulses;
	if(target_pulses_2 >= 32768) target_pulses_2 = target_pulses_2 - 32768;	
	//Indicate command
	while(current_pulses_1 != target_pulses_1 && current_pulses_2 != target_pulses_2){
		turn right
	}
	//Publish command to i90_position
	pub.publish(1);	

	/*Move i90*/	
	target_pos_x = targetValues.data[0]; //Get target position in 'x' axis from i90_sensor_board
	target_pos_y = targetValues.data[1]; //Get target position in 'y' axis from i90_sensor_board
	distance = sqrt(pow(target_pos_x-current_pos_x,2)+pow(target_pos_y-current_pos_y,2)) //Calculate distance in meters between target and current positions
	distance_pulses = int (distance / 0.00066); //Culculate quantity of encoder's pulses to reach that distance
	//Calculate target pulses	
	target_pulses_1 = current_pulses_1 + distance_pulses;
	if(target_pulses_1 >= 32768) target_pulses_1 = target_pulses_1 - 32768;
	target_pulses_2 = current_pulses_2 + distance_pulses;
	if(target_pulses_2 <= 0) target_pulses_1 = - 32768 - target_pulses_2;	
	//Indicate command	
	while(current_pulses_1 != target_pulses_1 && current_pulses_2 != target_pulses_2){
		go forward;
	} 
	//Publish command to i90_position
	pub.publish(1);

	ROS_INFO("Target values: [%f]/t[%f]", targetValues.data[0], targetValues.data[1]);
}

void get_current_pos(const std_msgs::Float32MultiArray::ConstPtr& targetValues){
	current_pos_x = targetValues.data[0]; //Change this
	current_pos_y = targetValues.data[1]; //Change this

	ROS_INFO("Target values: [%f]/t[%f]", targetValues.data[0], targetValues.data[1]);
}
