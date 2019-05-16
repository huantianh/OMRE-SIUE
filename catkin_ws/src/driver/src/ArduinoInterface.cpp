#include "driver/ArduinoInterface.h"

#include <sstream>
#include <iostream>


ArduinoInterface::ArduinoInterface(std::string port, int baud, int timeout) 
: arduino(port, baud, serial::Timeout::simpleTimeout(timeout))
{
    // Enable arduino
    ros::Duration(5.0).sleep();
    //std::string initString = arduino.readline(256, "\r");
    //arduino.write("H 0\r");
}

ArduinoInterface::~ArduinoInterface() {
    disable();
    arduino.flush();
    arduino.close();
}

void ArduinoInterface::enable(){
	arduino.write("H 0\r");
}

void ArduinoInterface::disable(){
	arduino.write("H 1\r");
}

// Read the state from arduino and return states as RobotInfo message
arduino_msgs::RobotInfo ArduinoInterface::read() {
    std::stringstream cmdss;
    std::stringstream rcss;
    arduino.flushOutput();
    arduino.flushInput();

    arduino_msgs::RobotInfo robotStatus;
    
    cmdss.str("");
    rcss.str("");
    int integerData = 0;
    double floatData = 0.0;
    

    // Read econders==========================================
    for(int i = 0; i < 3; i++){
		// Write commands to arduino
		cmdss << "E " << i << "\r";
		arduino.write(cmdss.str());

		// Read serials from arduino
		rcss.str(arduino.readline(256, "\r"));
		rcss >> integerData;
		//std::cout << integerData << std::endl;
		robotStatus.enconder.push_back(integerData);

		// Clean streams for safety
		cmdss.str("");
    	rcss.str("");
    }

/*

    // Read ultrasonic sensors=================================
    for(int i = 1; i < 6; i++){
	// Write commands to arduino
	cmdss << "U " << i << "\r";
	arduino.write(cmdss.str());

	// Read serials from arduino
	rcss.str(arduino.readline(256, "\r"));
	rcss >> floatData;
	std::cout << floatData << std::endl;
	//robotStatus.ultraSonic.push_back((float)floatData);
	
	// Clean streams for safety
	cmdss.str("");
    	rcss.str("");
    }
*/
/*
   // Read IR sensors=========================================
    for(int i = 0; i < 4; i++){
	// Write commands to arduino
	cmdss << "I " << (i+1) << "\r";
	arduino.write(cmdss.str());

	// Read serials from arduino
	rcss.str(arduino.readline(256, "\r"));
	rcss >> floatData;
	//robotStatus.ir.push_back((float)floatData);
	
	// Clean streams for safety
	cmdss.str("");
    	rcss.str("");
    }
*/
    return robotStatus;
}

// Push commands to the robot
void ArduinoInterface::write() {
    std::stringstream serialString;
    std::string temp = "v 200 200 200";

    //Recieve command from ROS
    //std::cout << "Enter command: ";
    //std::getline(std::cin,temp);

    //Send message to arduino
    serialString << temp << " \r";
    arduino.write(serialString.str());
    ROS_INFO_STREAM(serialString.str());
    serialString.str("");

}

