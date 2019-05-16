#ifndef ARDUINO_INTERFACE_H
#define ARDUINO_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <serial/serial.h>

//Messages ==============================
#include <arduino_msgs/RobotInfo.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

class ArduinoInterface : public hardware_interface::RobotHW {
	public:
	    ArduinoInterface(std::string port, int baud, int timeout);
	    ~ArduinoInterface();

	    arduino_msgs::RobotInfo read();
	    void write();

	    void enable();
	    void disable();


	private:
	    serial::Serial arduino;
	    hardware_interface::JointStateInterface stateInterface;
	    hardware_interface::VelocityJointInterface velocityCommandInterface;
	    hardware_interface::PositionJointInterface positionCommandInterface;

	    double pos[4];
	    double vel[4];
	    double eff[4];

	    double cmdPos[4] = {0};
	    double cmdVel[4] = {0};
	    double cmdEff[4] = {0};

};

#endif
