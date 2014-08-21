///////////////////////////////////////////////////////////////////////////////
//Source for i90_movement node to send movement commands to i90							 //
//v1.4 																																			 //
//Based on:																																	 //
//v1.3																																			 //
//Changelog:																																 //
//-Speed decreased to 0.2																										 //
//Huseyin Emre Erdem 																												 //
//20.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////

/*This node reads the target position information published from the i90_sensor node
under i90_target_pos topic. The type of message published is i90_movement::pos.
After acquiring the target, it turns and goes to the target. Informs other nodes upon
competion.
During turning it checks for left and right ir values to prevent hitting object.*/

#include "ros/ros.h"
#include "pos.h"//position information
#include "MotorInfoArray.h"//encoder readings
#include "RangeArray.h"//For i90 ir readings
#include <math.h>
#include "UInt8.h"
#include <geometry_msgs/Twist.h>//For movement commands
#include <unistd.h>//for usleep()

#define MAXENCODER 32768//Incremantal encoder limit
#define PI 3.141593
#define SPEED 0.1//Default: 0.5
#define DURTRANSLATION 2400000//Duration for 0.5m travel with predefined speed (1300000 for 0.5 speed, 2400000 for 0.1)
#define DURTURN 3200000//Duration for 90 degrees of turning with predefined speed (2500000 for 0.5 speed, 3200000 for 0.1)

/*Variables*/
float fCurrentPosX;//Current X position
float fCurrentPosY;//Current Y position
float fCurrentAngleYaw;//Current heading angle
float fTargetPosX;//Target X position
float fTargetPosY;//Target Y position
float fTargetAngleYaw;//Target heading
float fDistance;//Difference between the target current positions
float fAngle;//Difference between the target yaw and the current yaw angles
float fI90IrL;//Distance value of Ir sensor fixed on i90 body
float fI90IrR;//Distance valeu of Ir sensor fixed on i90 body
useconds_t duration;//Duration to give motor control orders to limit the movement amount
int iCurrentPulseL; //Encoder pulse value for the left motor
int iCurrentPulseR; //Encoder pulse value for the right motor
int iTargetPulseL;//Encoder pulse target value for the left motor
int iTargetPulseR;//Encoder pulse target value for the right motor
int iTurnPulses;//Amount of pulses to turn to correct heading
int iDistancePulses;//Amount of pulses to move to correct position
bool bTurnState;//Shows if the turning before straight movement is done or not, 0:not 1:done
bool bTurnDir;//0:CW, 1:CCW
i90_movement::UInt8 iRotationFlag;//Flag to be used in rotationPub
i90_movement::UInt8 iTranslationFlag;//Flag to be used in translationPub
geometry_msgs::Twist cmdvel_;//commands to be sent to i90

/*Prototypes*/
void turnToTargetPos(const i90_movement::pos posReceived);//Turns i90 to target position
void moveToTargetPos(void);//Moves i90 to target position
void getCurrentPos(const i90_movement::pos posReceived);//Processes data read from i90_current_pos topic
void getEncoderPulses(const drrobot_I90_player::MotorInfoArray MotorInfoArray);//Processes data read from drrobot_motor topic
void readI90Ir(const drrobot_I90_player::RangeArray i90Ir);

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
	ros::Subscriber irPub = n.subscribe("drrobot_ir",1,readI90Ir); 

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
			if(bTurnDir == 1){

				/*Check for side ir distance sensors to prevent hitting obstacles during turning*/
				duration = round(duration / 10);//Divide duration to 10 to check side distance 10 times
				int iCounter = 0;
				cmdvel_.angular.z = SPEED;
				velPub.publish(cmdvel_);//Send turn command to i90
				while(fI90IrL > 0.1 && iCounter < 10){//If distance is more than 0.1m, keep turning
					usleep(duration);
					iCounter++;
				}
			}

			/*Turning cw*/
			else{

				/*Check for side ir distance sensors to prevent hitting obstacles during turning*/
				duration = round(duration / 10);//Divide duration to 10 to check side distance 10 times
				int iCounter = 0;
				cmdvel_.angular.z = -SPEED;
				velPub.publish(cmdvel_);//Send turn command to i90
				while(fI90IrR > 0.1 && iCounter < 10){//If distance is more than 0.1m, keep turning
					usleep(duration);
					iCounter++;
				}
			}

			/*Stop*/			
			cmdvel_.angular.z = 0.00;
			velPub.publish(cmdvel_);//Stop
			rotationPub.publish(iRotationFlag);//Inform other nodes to estimate the new heading
			iRotationFlag.data = 0;
		}

		/*Going straight*/
		if(bTurnState == true){
			usleep(1000000);//Wait to allow position estimation performed
			moveToTargetPos();
			ROS_INFO("Translation Duration: %d", duration);
			cmdvel_.linear.x = SPEED;
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
	ROS_INFO("Turning target values: %f\t%f\t%f", fTargetPosX, fTargetPosY, fTargetAngleYaw);

	fAngle = fCurrentAngleYaw - fTargetAngleYaw;
	if(fAngle < 0.00){
		fAngle = abs(fAngle);
		if(fAngle > 90.00){
			fAngle = 360.00 - fAngle;
			bTurnDir = 0;
		}
		else{
			bTurnDir = 1;
		}
	}
	else{
		if(fAngle > 90.00){
			fAngle = 360.00 - fAngle;
			bTurnDir = 1;
		}
		else{
			bTurnDir = 0;
		}
	}

	duration = fAngle / 90.00 * DURTURN;//90.00 degrees ~= 2500000 uSeconds	with SPEED angular x speed	
	ROS_INFO("Rotation Duration: %d", duration);
	ROS_INFO("Angle: %f\t%d", fAngle, bTurnDir);
	iRotationFlag.data = 1;//Allow sending turn commands to the robot

	bTurnState = true;
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

	duration = fDistance / 0.5 * DURTRANSLATION;//0.5m with 0.5 linear speed ~= 1300000
	ROS_INFO("Translation Duration: %d", duration);

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

void readI90Ir(const drrobot_I90_player::RangeArray i90Ir){
	fI90IrL = i90Ir.ranges[0].range;//Leftmost Ir sensor. Refer to the coding report for all the sensor numbers.
	fI90IrR = i90Ir.ranges[6].range;//Rightmost Ir sensor
}

