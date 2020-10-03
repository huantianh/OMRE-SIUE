import omrebot
from omrebot import OmreBot
import libomni as robot1  #Library that handles all the serial commands to arduino AtMega


robot = OmreBot('/dev/ttyACM0')


robot.port = "ha"

mode = str(input("Enter mode. s for serial, t for input tester, c for controller, o, "))

if(mode == 's'):

############## Simple Serial Communicator to Arduino ##############
	while True:
		command = input("Enter Command: ")
		command = command+'\r'
		robot.ser.write(command.encode())
		print (robot.ser.readline().decode("ascii"))
	# ~ robot.motor_rpm(0,50,-50)

# ~ robot.motor_rpm(0,100,-100)
