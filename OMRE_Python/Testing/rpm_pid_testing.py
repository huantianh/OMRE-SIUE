import serial
import math, time, os
import numpy as np
import sys
import libomni as robot

ser = serial.Serial('/dev/ttyACM0',115200, timeout=.4);


ser.reset_input_buffer()
ser.reset_output_buffer()

#folder where saving all the data
save_folder = "Data_Testing/RPM_PID_Testing/"
current_directory = os.getcwd()
save_directory = os.path.join(current_directory, save_folder)

def readEncoder(encoderNum):
	ser.reset_input_buffer()
	ser.write(("e %d \r" % (encoderNum)).encode())
	encoderValue = (ser.readline().decode())
	data = float(encoderValue.strip())
	return data
	
def PWMValues(pwm):
	ser.reset_input_buffer()
	for x in range(3):
		ser.write(("m %s %s %s \r" % (x,abs(int(pwm)),0)).encode())

def PIDValues(Kp,Ki,Kd):
	ser.reset_input_buffer()
	ser.write(("k %s %s %s \r" % (Kp,Ki,Kd)).encode())
	
def PIValues_3(Kp1,Kp2,Kp3,Ki1,Ki2,Ki3):
	ser.reset_input_buffer()
	ser.write(("w %s %s %s %s %s %s \r" % (Kp1,Kp2,Kp3,Ki1,Ki2,Ki3)).encode())
	
def enablePrint(printEnable):
	print_setup = printEnable
	ser.write(("p %d \r" % (print_setup)).encode())
	
def RPM_Values(rpm):
	ser.reset_input_buffer()
	ser.write(("v %d %d %d \r" % (int(rpm),int(rpm),int(rpm))).encode())	
	
def motors(m1,m2,m3):
	motorValues = [m1,m2,m3]
	for x in range(3):
		ser.write(("m %d %d %d\r" % (x, abs(motorValues[x]), int(motorValues[x]>=0))).encode())



try: 
	mode = str(input("Enter mode: s for serial, t for RPM testing "))

	if(mode == 's'):

	############## Simple Serial Communicator to Arduino ##############
		while True:
			command = input("Enter Command: ")
			command = command+'\r'
			ser.write(command.encode())
			print (ser.readline().decode())


	elif mode == 't':
		
		#~ motor_num = int(input("enter motor number: "))
		f = open("rpm_values.txt",'r')
		lines = f.readlines()
		timer = []
		rpm = []
		printSetup = 1
		enablePrint(printSetup)
		
		for line in lines:
			x = line.split(',')[0]
			y = line.split(',')[1]
			timer = x	
			rpm = y
			RPM_Values(int(rpm))
			
			#~ file = open(save_folder + "Motor: "+str(motor_num)+".txt","a")		
			file = open(save_folder + "All Motors: 1,2,3 "+".txt","a")		
			
			start = time.time()
		
			while time.time()-float(start) <= float(timer):
				data = [0]
				if(ser.inWaiting() > 0):				
					data = (ser.readline().decode())
					sys.stdout.write(data)
					sys.stdout.write(rpm)
					file.writelines(" {0} , {1}".format(data[:-2],rpm))
			file.close		
				
## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()     
	print('\n\n		Stop!!! See you again!')
		
		







