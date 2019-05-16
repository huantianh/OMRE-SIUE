#include "ros/ros.h"
#include <arduino_msgs/RobotInfo.h>
#include <iostream>

void updateRPM(const arduino_msgs::RobotInfo msg);

double pastEncoderValues[3]    = {0, 0, 0};
double changeInEncoders[3]    = {0, 0, 0};
double changeInRevolutions[3] = {0, 0, 0};
double changeInTimeSeconds[3] = {0, 0, 0};
int rpmValues[3];
ros::Time now;
ros::Time pastTimes;


int main(int argc, char **argv){
	ros::init(argc, argv, "rpmChecker");
	ros::NodeHandle nh;
	
	// Initialize pastTime at start up once
	pastTimes = ros::Time::now();
	
	// This is in its own scope because pastTimes value is meant
	// to only be intialized once before going into the loop
	{
		ros::Subscriber sub = nh.subscribe("/robotState", 1, updateRPM);
		ros::spin();
	}
	return 0;
}


// This function is called every time new message is update to /robotState topic
void updateRPM(const arduino_msgs::RobotInfo msg)
{
  now = ros::Time::now();
  for ( int i = 0; i < 3; i++)
  {
    changeInEncoders[i] = msg.enconder[i] - pastEncoderValues[i];
    changeInTimeSeconds[i] = now.toSec() - pastTimes.toSec();
    changeInRevolutions[i] = changeInEncoders[i] / 2248.6;

    rpmValues[i] = (changeInRevolutions[i] / (changeInTimeSeconds[i])) * 60; // *60 to get Revolutions per MINUTE

    // update our values to be used next time around
    pastEncoderValues[i] = msg.enconder[i];

    std::cout << rpmValues[i] << std::endl;
  }
  pastTimes = ros::Time::now();
}
