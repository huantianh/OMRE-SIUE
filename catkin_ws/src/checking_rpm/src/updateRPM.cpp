#include "ros/ros.h"
#include <arduino_msgs/RobotInfo.h>
#include <iostream>

void updateRPM(const arduino_msgs::RobotInfo msg);

float pastEncoderValues[3]    = {0, 0, 0};
unsigned long pastTimes       =  0;
double changeInEncoders[3]    = {0, 0, 0};
double changeInRevolutions[3] = {0, 0, 0};
double changeInTimeSeconds[3] = {0, 0, 0};
int rpmValues[3];
unsigned long now=0;



int main(int argc, char **argv){
	ros::init(argc, argv, "rpmChecker");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("/robotState", 1, updateRPM);
	
	//ros::Duration(1).sleep();
	
	ros::spin();
	return 0;
}



void updateRPM(const arduino_msgs::RobotInfo msg)
{
  for ( int i = 0; i < 3; i++)
  {
	now = ros::Time::now().toSec();
    changeInEncoders[i] = msg.enconder[i] - pastEncoderValues[i];
    changeInTimeSeconds[i] = (now - pastTimes); // *.001 to convert to seconds
    changeInRevolutions[i] = changeInEncoders[i] / 2248.6;

    rpmValues[i] = (changeInRevolutions[i] / (changeInTimeSeconds[i])) * 60; // *60 to get Revolutions per MINUTE

    // update our values to be used next time around
    pastTimes = now;
    pastEncoderValues[i] = msg.enconder[i];
    std::cout << ros::Time::now() << std::endl;
    //std::cout << "Encoder " << i << " " << msg.enconder[i] << std::endl;
    //std::cout << "ChangeInRevolutions " << i << " " << changeInRevolutions[i] << std::endl;
  }
}
