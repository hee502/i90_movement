///////////////////////////////////////////////////////////////////////////////
//Source for i90_movement node to send movement commands to i90							 //
//v1.3 																																			 //
//Based on:																																	 //
//v1.2																																			 //
//Changelog:																																 //
//-Works in accordance with the global coordinate frame											 //
//Huseyin Emre Erdem 																												 //
//16.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////

/*This node reads the target position information published from the i90_sensor node
under i90_target_pos topic. The type of message published is i90_movement::pos.
After acquiring the target, it turns and goes to the target. Informs other nodes upon
competion.*/

#include "ros/ros.h"
#include "pos.h"//position information
#include "MotorInfoArray.h"//encoder readings
#include <math.h>
#include "UInt8.h"
#include <geometry_msgs/Twist.h>//For movement commands
#include <unistd.h>//for usleep()

#define MAXENCODER 32768//Incremantal encoder limit
#define PI 3.141593

/*Variables*/
float fCurrentPosX;//Current X position
float fCurrentPosY;//Current Y position
float fCurrentAngleYaw;//Current heading angle
float fTargetPosX;//Target X position
float fTargetPosY;//Target Y position
float fTargetAngleYaw;//Target heading
float fDistance;//Difference between the target current positions
float fAngle;//Difference between the target yaw and the current yaw angles
useconds_t duration;//Duration to give motor control orders to limit the movement amount
int iCurrentPulseL; //Encoder pulse value for the left motor
int iCurrentPulseR; //Encoder pulse value for the right motor
int iTargetPulseL;//Encoder pulse target value for the left motor
int iTargetPulseR;//Encoder pulse target value for the right motor
int iTurnPulses;//Amount of pulses to turn to correct heading
int iDistancePulses;//Amount of pulses to move to correct position
bool bTurnState;//Shows if the turning before straight movement is done or not, 0:not 1:done

i90_movement::UInt8 iRotationFlag;//Flag to be used in rotationPub
i90_movement::UInt8 iTranslationFlag;//Flag to be used in translationPub
geometry_msgs::Twist cmdvel_;//commands to be sent to i90

/*Prototypes*/
void turnToTargetPos(const i90_movement::pos posReceived);//Turns i90 to target position
void moveToTargetPos(void);//Moves i90 to target position
void getCurrentPos(const i90_movement::pos posReceived);//Processes data read from i90_current_pos topic
void getEncoderPulses(const drrobot_I90_player::MotorInfoArray MotorInfoArray);//Processes data read from drrobot_motor topic

/*Main function*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "i90_movement");//Create node called "i90_movement"
	ros::NodeHandle n;//Create nodehandler to modify features of the node
	ros::Subscriber targetSub = n.subscribe("i90_target_pos", 1, turnToTargetPos);//To read target positions
	ros::Subscriber currentSub = n.subscribe("i90_current_pos", 1, getCurrentPos);//To read current estimated position
	ros::Subscriber encoderSub = n.subscribe("drrobot_motor", 1, getEncoderPulses);//To read encoder values
	ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("drrobot_cmd_vel", 1);//To publish movement commands
	ros::Publisher rotationPub = n.advertise<i90_movement::UInt8>("i90_rotation_done",1);//To publish flags indicating rotation is done (to update yaw angle)
	ros::Publisher translationPub = n.advertise<i90_movement::UInt8>("i90_translation_done",1);//To publish flags indicating movement is done (to update position)

	ros::Duration d = ros::Duration(2,0);
	ros::Rate loop_rate(2);//2 Hz
	//geometry_msgs::Twist cmdvel_;//for movement commands to be sent to i90
	iRotationFlag.data = 0;//Set flag to 1
	//std_msgs::bool bTranslationFlag;//Flag to be used in translationPub & rotationPub
	//bTranslationFlag.data = false;//Set flag to 1
	iTranslationFlag.data = 1;

	while (ros::ok()){
		/*cmdvel_.linear.x = 0.1;
		ROS_INFO("Speed: %f", cmdvel_.linear.x);
		pub.publish(cmdvel_);*/
		// d.sleep();

		/*Check flags to publish messages*/
		if(iRotationFlag.data == 1){//If calculations are done to send the commands to the robot

			/*Turning ccw*/
			if(duration < 5000000){
				cmdvel_.angular.z = 0.5;
				velPub.publish(cmdvel_);//Send turn command to i90
				usleep(duration);
			}

			/*Turning cw*/
			else{
				duration = 10000000 - duration;				
				cmdvel_.angular.z = -0.5;
				velPub.publish(cmdvel_);//Send turn command to i90
				usleep(duration);
			}
			
			/*Stop*/			
			cmdvel_.angular.z = 0.00;
			velPub.publish(cmdvel_);//Stop
			rotationPub.publish(iRotationFlag);//Inform other nodes to estimate the new heading
			iRotationFlag.data = 0;
		}

		/*Going straight*/
		if(bTurnState == true){
			usleep(100000);//Wait to allow position estimation performed
			moveToTargetPos();

			cmdvel_.linear.x = 0.5;
			velPub.publish(cmdvel_);//Send movement command to i90
			usleep(duration);
			cmdvel_.linear.x = 0.0;//Stop the robot
			velPub.publish(cmdvel_);
			translationPub.publish(iTranslationFlag);//Inform other nodes to estimate the new position
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

/*Processes data read from i90_target_pos topic*/
void turnToTargetPos(const i90_movement::pos posReceived){
	fTargetPosX = posReceived.fXPos; //Get target position in 'x' axis from i90_sensor_board
	fTargetPosY = posReceived.fYPos; //Get target position in 'y' axis from i90_sensor_board
	fTargetAngleYaw = posReceived.fYawAngle;//Get target angle

	fAngle = fTargetAngleYaw - fCurrentAngleYaw;

	duration = fAngle / 90.00 * 2500000;//90.00 degrees ~= 2500000 uSeconds	with 0.5 angular x speed	
	ROS_INFO("Angle: %f", fAngle);
	iRotationFlag.data = 1;//Allow sending turn commands to the robot
	
	bTurnState = true;
	ROS_INFO("Turning target values: %f\t%f\t%f", fTargetPosX, fTargetPosY, fTargetAngleYaw);
}

/*Runs after turnToTargetPos and moves until reaching the target*/
void moveToTargetPos(void){
	fDistance = sqrt(pow(fTargetPosX-fCurrentPosX,2)+pow(fTargetPosY-fCurrentPosY,2)); //Calculate distance in meters between target and current positions
	iDistancePulses = int (fDistance / 0.00066); //Culculate quantity of encoder's pulses to reach that distance
	
	ROS_INFO("Distance: %f", fDistance);
	iTargetPulseL = iCurrentPulseL + iDistancePulses;
	if(iTargetPulseL >= MAXENCODER) iTargetPulseL = iTargetPulseL - MAXENCODER;
	iTargetPulseR = iCurrentPulseR + iDistancePulses;
	if(iTargetPulseR <= 0) iTargetPulseL = - MAXENCODER - iTargetPulseR;

	duration = fDistance / 0.5 * 1300000;//0.5m with 0.5 linear speed ~= 1300000
	ROS_INFO("Speed: %f", cmdvel_.linear.x);

	ROS_INFO("Moving target values: %f\t%f\t%f", fTargetPosX, fTargetPosY, fTargetAngleYaw);
	bTurnState = false;
}

/*Processes data read from i90_current_pos topic*/
void getCurrentPos(const i90_movement::pos posReceived){
	fCurrentPosX = posReceived.fXPos;
	fCurrentPosY = posReceived.fYPos;
	fCurrentAngleYaw = posReceived.fYawAngle;
	//ROS_INFO("Current position: [%f]/t[%f]", targetValues.fXPos, targetValues.fYPos);*/
}

/*Processes data read from drrobot_motor topic*/
void getEncoderPulses(const drrobot_I90_player::MotorInfoArray motorInfoArray)
{
	iCurrentPulseL =	motorInfoArray.motorInfos[0].encoder_pos;
	iCurrentPulseR =	motorInfoArray.motorInfos[1].encoder_pos;
	//ROS_INFO("Encoders: [%u]/t[%u]", motorInfoArray.motorInfos[0].encoder_pos, motorInfoArray.motorInfos[1].encoder_pos);
}

