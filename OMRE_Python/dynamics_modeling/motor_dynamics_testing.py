import serial
import math, time, os
import numpy as np
import sys
import libomni as robot

ser = serial.Serial('/dev/ttyACM0',115200, timeout=.4);
ser.reset_input_buffer()
ser.reset_output_buffer()

#folder where saving all the data
save_folder = "save_data/"
current_directory = os.getcwd()
save_directory = os.path.join(current_directory, save_folder)
	

####################################################################
def motor_pwm(pwm):
	ser.reset_input_buffer()
	for x in range(3):
		ser.write(("m %d %d \r" % (x,abs(int(pwm)))).encode())

def motor0_pwm(m1):
	motorValues = m1
	x = 0
	ser.write(("m %d %d \r" % (x, motorValues)).encode())

def rpm(rpmNum):
	ser.reset_input_buffer()
	ser.write(("r %f \r" % (rpmNum)).encode())
	rpmValue = (ser.readline().decode("ascii"))
	return float(rpmValue.rstrip())
	
def current_sensor(curNum):
	ser.reset_input_buffer()
	ser.write(("o r").encode())
	curValue = (ser.readline().decode("ascii"))
	return float(curValue.rstrip())	
	
def motor_voltage():
	ser.reset_input_buffer()
	ser.write(("t \r").encode())
	volValue = (ser.readline().decode("ascii"))
	return float(volValue.rstrip())		

def enablePrint():
	ser.write(("p \r").encode())

def stop():
	ser.write(("s \r").encode())	




try: 
	mode = str(input("Enter mode: s to start testing  "))

	if(mode == 's'):

		f = open("pwm_values.txt",'r')
		lines = f.readlines()
		timer = []
		pwm = []
		enablePrint()
		
		for line in lines:
			x = line.split(',')[0]
			y = line.split(',')[1]
			timer = x	
			pwm = y
			
			motor_pwm(int(pwm))
		
			
			file = open(save_folder + "Motors_dynamics_test"+".txt","a")		
			
			start = time.time()
		
			while time.time()-float(start) <= float(timer):
				data = [0]
				if(ser.inWaiting() > 0):				
					data = (ser.readline().decode())
					sys.stdout.write(data)
					sys.stdout.write(pwm)
					file.writelines(" {0} , {1}".format(data[:-2],pwm))
			file.close		
				
## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()     
	print('\n\n		Stop!!! See you again!')
