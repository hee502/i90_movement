///////////////////////////////////////////////////////////////////////////////
//Source for i90_movement node to send movement commands to i90							 //
//v1.2 																																			 //
//Changelog:																																 //
//-Compilation errors are solved.																						 //
//Huseyin Emre Erdem 																												 //
//Edited by:																																 //
//Edgar Buchanan Berumen																										 //
//14.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////

/*This node reads the target position information published from the i90_sensor node
under i90_target_pos topic. The type of message published is i90_movement::pos.
After acquiring the target, it turns and goes to the target. Informs other nodes upon
competion.*/

#include "ros/ros.h"
//#include <geometry_msgs/Twist.h>//movement commands
#include "pos.h"//position information
#include "MotorInfoArray.h"//encoder readings
#include <math.h>
#include "UInt8.h"
#include <geometry_msgs/Twist.h>//For movement commands

#define MAXENCODER 32768//Incremantal encoder limit
#define PI 3.141593

/*Variables*/
float fCurrentPosX;
float fCurrentPosY;
float fTargetPosX;
float fTargetPosY;
float fDistance;
float fSlope;
float fAngle;
int iTurnPulses;
int iDistancePulses;
int iCurrentPulseL; //Ask some node to retrieve encoders information
int iCurrentPulseR; //Ask some node to retrieve encoders information
int iTargetPulseL;
int iTargetPulseR;

i90_movement::UInt8 iRotationFlag;//Flag to be used in rotationPub
i90_movement::UInt8 iTranslationFlag;//Flag to be used in translationPub
geometry_msgs::Twist cmdvel_;//commands to be sent to i90

/*Prototypes*/
void moveToTargetPos(const i90_movement::pos posReceived);//Processes data read from i90_target_pos topic
void getCurrentPos(const i90_movement::pos posReceived);//Processes data read from i90_current_pos topic
void getEncoderPulses(const drrobot_I90_player::MotorInfoArray MotorInfoArray);//Processes data read from drrobot_motor topic

/*Main function*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "i90_movement");//Create node called "i90_movement"
	ros::NodeHandle n;//Create nodehandler to modify features of the node
	ros::Subscriber targetSub = n.subscribe("i90_target_pos", 1, moveToTargetPos);//To read target positions
	ros::Subscriber currentSub = n.subscribe("i90_current_pos", 1, getCurrentPos);//To read current estimated position
	ros::Subscriber encoderSub = n.subscribe("drrobot_motor", 1, getEncoderPulses);//To read encoder values
	ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("drrobot_cmd_vel", 1);//To publish movement commands
	ros::Publisher rotationPub = n.advertise<i90_movement::UInt8>("i90_rotation_done",1);//To publish flags indicating rotation is done (to update yaw angle)
	ros::Publisher translationPub = n.advertise<i90_movement::UInt8>("i90_translation_done",1);//To publish flags indicating movement is done (to update position)

	ros::Duration d = ros::Duration(2,0);
	ros::Rate loop_rate(2);//2 Hz
	//geometry_msgs::Twist cmdvel_;//for movement commands to be sent to i90
	iRotationFlag.data = 0;//Set flag to 1
//	std_msgs::bool bTranslationFlag;//Flag to be used in translationPub & rotationPub
	//bTranslationFlag.data = false;//Set flag to 1

	while (ros::ok()){
		/*cmdvel_.linear.x = 0.1;
		ROS_INFO("Speed: %f", cmdvel_.linear.x);
		pub.publish(cmdvel_);*/
		// d.sleep();

		/*Check flags to publish messages*/
		if(iRotationFlag.data == 1){
			velPub.publish(cmdvel_);//Send turn command to i90
			rotationPub.publish(iRotationFlag);//Inform other nodes to estimate the new heading
			iRotationFlag.data = 0;
		}
		if(iTranslationFlag.data == 1){
			velPub.publish(cmdvel_);//Send movement command to i90
			translationPub.publish(iTranslationFlag);//Inform other nodes to estimate the new position
			iTranslationFlag.data = 0;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

/*Processes data read from i90_target_pos topic*/
void moveToTargetPos(const i90_movement::pos posReceived){
	fTargetPosX = posReceived.fXPos; //Get target position in 'x' axis from i90_sensor_board
	fTargetPosY = posReceived.fYPos; //Get target position in 'y' axis from i90_sensor_board
	ROS_INFO("Target: %f\t%f", fTargetPosX, fTargetPosY);

	fSlope = (fTargetPosY-fCurrentPosY)/(fTargetPosX-fCurrentPosX); //Calculate slope between target and current positions
	fAngle = atan2 (fTargetPosY-fCurrentPosY, fTargetPosX-fCurrentPosX) * 180 / PI; //Calculate angle in degrees between target and current positions
	iTurnPulses = int (fAngle / 0.235); //Calculate quantity of encoder's pulses to reach angle.

	iTargetPulseL = iCurrentPulseL + iTurnPulses;
	if(iTargetPulseL >= MAXENCODER) iTargetPulseL = iTargetPulseL - MAXENCODER;
	iTargetPulseR = iCurrentPulseR - iTurnPulses;
	if(iTargetPulseR >= MAXENCODER) iTargetPulseR = iTargetPulseR - MAXENCODER;

	while(iCurrentPulseL != iTargetPulseL && iCurrentPulseR != iTargetPulseR){
		cmdvel_.angular.z = 0.1;
		ROS_INFO("Speed: %f", cmdvel_.linear.x);
		//turn right
	}

	iRotationFlag.data = 1;

	usleep(100000);//Wait to allow position estimation performed

	fDistance = sqrt(pow(fTargetPosX-fCurrentPosX,2)+pow(fTargetPosY-fCurrentPosY,2)); //Calculate distance in meters between target and current positions
	iDistancePulses = int (fDistance / 0.00066); //Culculate quantity of encoder's pulses to reach that distance

	iTargetPulseL = iCurrentPulseL + iDistancePulses;
	if(iTargetPulseL >= MAXENCODER) iTargetPulseL = iTargetPulseL - MAXENCODER;
	iTargetPulseR = iCurrentPulseR + iDistancePulses;
	if(iTargetPulseR <= 0) iTargetPulseL = - MAXENCODER - iTargetPulseR;

	while(iCurrentPulseL != iTargetPulseL && iCurrentPulseR != iTargetPulseR){
		cmdvel_.linear.x = 0.1;
		ROS_INFO("Speed: %f", cmdvel_.linear.x);
		//go forward;
	}

	iTranslationFlag.data = 1;

	//ROS_INFO("Reached target values: [%f]/t[%f]", targetValues.fXPos, targetValues.fYPos);
}

/*Processes data read from i90_current_pos topic*/
void getCurrentPos(const i90_movement::pos posReceived){
	fCurrentPosX = posReceived.fXPos;
	fCurrentPosY = posReceived.fYPos;
	//ROS_INFO("Current position: [%f]/t[%f]", targetValues.fXPos, targetValues.fYPos);*/
}

/*Processes data read from drrobot_motor topic*/
void getEncoderPulses(const drrobot_I90_player::MotorInfoArray motorInfoArray)
{
	iCurrentPulseL =	motorInfoArray.motorInfos[0].encoder_pos;
	iCurrentPulseR =	motorInfoArray.motorInfos[1].encoder_pos;
	//ROS_INFO("Encoders: [%u]/t[%u]", motorInfoArray.motorInfos[0].encoder_pos, motorInfoArray.motorInfos[1].encoder_pos);
}

