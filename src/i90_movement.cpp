///////////////////////////////////////////////////////////////////////////////
//Source for i90_movement node to send movement commands to i90							 //
//v1.5 																																			 //
//Changelog:																																 //
//-3 step movement added (go 20 cm forward, turn, go 50 cm forward)				   //
//-Internal IRs cancelled
//-Step numbers added
//ToDo:																																			 //
//-Add movement with 3 steps																								 //
//Huseyin Emre Erdem 																												 //
//22.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////

/*This node makes i90 go to the target position information published from i90_sensor node
under i90_target_pos topic. The type of message published is i90_movement::pos.
After acquiring the target, it moves 20 cm forward to make the center of the robot
and the sensors equal, turns and goes to the target. Informs other nodes upon
competion.*/

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
volatile float fCurrentPosX;//Current X position
volatile float fCurrentPosY;//Current Y position
volatile float fCurrentAngleYaw;//Current heading angle
volatile float fTargetPosX;//Target X position
volatile float fTargetPosY;//Target Y position
volatile float fTargetAngleYaw;//Target heading
volatile float fDistance;//Difference between the target current positions
volatile float fAngle;//Difference between the target yaw and the current yaw angles
useconds_t duration;//Duration to give motor control orders to limit the movement amount
volatile int iCurrentPulseL; //Encoder pulse value for the left motor
volatile int iCurrentPulseR; //Encoder pulse value for the right motor
volatile int iTargetPulseL;//Encoder pulse target value for the left motor
volatile int iTargetPulseR;//Encoder pulse target value for the right motor
volatile int iTurnPulses;//Amount of pulses to turn to correct heading
volatile int iDistancePulses;//Amount of pulses to move to correct position
volatile bool bTurnState;//Shows if the turning before straight movement is done or not, 0:not 1:done
volatile bool bTurnDir;//0:CW, 1:CCW
i90_movement::UInt8 iRotationFlag;//Flag to be used in rotationPub
i90_movement::UInt8 iTranslationFlag;//Flag to be used in translationPub
geometry_msgs::Twist cmdvel_;//commands to be sent to i90
int iCounter = 0;

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

	/*Subscribers and Publishers*/
	ros::Subscriber targetSub = n.subscribe("i90_target_pos", 1, turnToTargetPos);//To read target positions
	ros::Subscriber currentSub = n.subscribe("i90_current_pos", 1, getCurrentPos);//To read current estimated position
	ros::Subscriber encoderSub = n.subscribe("drrobot_motor", 1, getEncoderPulses);//To read encoder values
	ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("drrobot_cmd_vel", 1);//To publish movement commands
	ros::Publisher rotationPub = n.advertise<i90_movement::UInt8>("i90_rotation_done",1);//To publish flags indicating rotation is done (to update yaw angle)
	ros::Publisher translationPub = n.advertise<i90_movement::UInt8>("i90_translation_done",1);//To publish flags indicating movement is done (to update position)

	ros::Duration d = ros::Duration(2,0);
	ros::Rate loop_rate(2);//2 Hz
	iRotationFlag.data = 0;//Set flag to 1
	iTranslationFlag.data = 1;

	while (ros::ok()){

		/*Check flags to publish messages*/
		if(iRotationFlag.data == 1){//If calculations are done to send the turning commands to the robot

			/*Turning ccw*/
			if(bTurnDir == 1){
				cmdvel_.angular.z = SPEED;
				velPub.publish(cmdvel_);//Send turn command to i90
				usleep(duration);
			}

			/*Turning cw*/
			else{
				cmdvel_.angular.z = -SPEED;
				velPub.publish(cmdvel_);//Send turn command to i90
				usleep(duration);
			}

			/*Stop*/			
			cmdvel_.angular.z = 0.00;
			velPub.publish(cmdvel_);//Stop
			usleep(1000000);//Wait 1s to allow robot to physically stop and get up-to-date encoder values
			rotationPub.publish(iRotationFlag);//Inform other nodes to estimate the new heading
			iRotationFlag.data = 0;
		}

		/*Going straight*/
		if(bTurnState == true){
			usleep(1000000);//Wait to allow position estimation performed
			moveToTargetPos();
			cmdvel_.linear.x = SPEED;
			velPub.publish(cmdvel_);//Send movement command to i90
			usleep(duration);
			cmdvel_.linear.x = 0.0;//Stop the robot
			velPub.publish(cmdvel_);
			usleep(1000000);//Wait to allow robot to physically stop
			translationPub.publish(iTranslationFlag);//Inform other nodes to estimate the new position
			iCounter++;
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
	ROS_INFO("-%d- Received target: %f\t%f\t%f", iCounter, fTargetPosX, fTargetPosY, fTargetAngleYaw);

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
	ROS_INFO("-%d- Angle difference: %f\t%d", iCounter, fAngle, bTurnDir);
	iRotationFlag.data = 1;//Allow sending turn commands to the robot

	bTurnState = true;
}

/*Runs after turnToTargetPos and moves until reaching the target*/
void moveToTargetPos(void){
	fDistance = sqrt(pow(fTargetPosX-fCurrentPosX,2)+pow(fTargetPosY-fCurrentPosY,2)); //Calculate distance in meters between target and current positions
	iDistancePulses = int (fDistance / 0.00066); //Culculate quantity of encoder's pulses to reach that distance

	ROS_INFO("-%d- Distance to translate: %f", iCounter, fDistance);
	iTargetPulseL = iCurrentPulseL + iDistancePulses;
	if(iTargetPulseL >= MAXENCODER) iTargetPulseL = iTargetPulseL - MAXENCODER;
	iTargetPulseR = iCurrentPulseR + iDistancePulses;
	if(iTargetPulseR <= 0) iTargetPulseL = - MAXENCODER - iTargetPulseR;

	duration = fDistance / 0.5 * DURTRANSLATION;//0.5m with 0.5 linear speed ~= 1300000
	bTurnState = false;
}

/*Processes data read from i90_current_pos topic*/
void getCurrentPos(const i90_movement::pos posReceived){
	fCurrentPosX = posReceived.fXPos;
	fCurrentPosY = posReceived.fYPos;
	fCurrentAngleYaw = posReceived.fYawAngle;
}

/*Processes data read from drrobot_motor topic*/
void getEncoderPulses(const drrobot_I90_player::MotorInfoArray motorInfoArray)
{
	iCurrentPulseL =	motorInfoArray.motorInfos[0].encoder_pos;
	iCurrentPulseR =	motorInfoArray.motorInfos[1].encoder_pos;
}

