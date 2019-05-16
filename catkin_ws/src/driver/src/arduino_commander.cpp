#include <controller_manager/controller_manager.h>

#include "driver/ArduinoInterface.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "arduino_commander");
    ros::NodeHandle nh;
    std::string port;
    int baud;

    if (!ros::param::get("~port", port)) {
      port = "/dev/ttyACM0";
      ROS_WARN_STREAM("No port specified, using default: " << port);
    }

    if (!ros::param::get("~baudrate", baud)) {
      baud = 115200;
      ROS_WARN_STREAM("No baudrate specified, using default: " << baud);
    }

    ros::param::param<std::string>("port", port, "/dev/ttyACM0");
    ros::param::param<int>("baudrate", baud, 115200);

    ArduinoInterface interface(port, baud, 250);

   ros::Rate rate(10.0);

   while(ros::ok()){
	interface.write();
	rate.sleep();
   }
}
