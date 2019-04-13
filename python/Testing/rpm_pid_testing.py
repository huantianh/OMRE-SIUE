import serial
import math, time, os, libomni
import numpy as np
import sys


ser = serial.Serial('/dev/ttyACM0',9600, timeout=.4);


ser.reset_input_buffer()
ser.reset_output_buffer()
#joy = xbox.Joystick()

#folder where saving all the data
save_folder = "Data_Testing/RPM_PID_Testing/"
current_directory = os.getcwd()
save_directory = os.path.join(current_directory, save_folder)

def readEncoder(encoderNum):
	ser.reset_input_buffer()
	ser.write(("e %d \r" % (encoderNum)).encode())
	
	encoderValue = (ser.readline().decode())
	#print(encoderValue)
	data = float(encoderValue.strip())
	#print(data)
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
	
def enablePID(pidValue):
	pid = pidValue
	ser.write(("p %d \r" % (pid)).encode())
	
def RPM_Values(rpm):
	ser.reset_input_buffer()
	ser.write(("v %d %d %d \r" % (int(rpm),int(rpm),int(rpm))).encode())	
	
	
# Fun function that takes all 3 motor PWM value from -255  to 255 and interprets it 
# into the correct command to send to arduino. Remember with Python3  Pyserial pretty much
# expects everything  in forms of bytes so we encode it into a byte and decode the output from
# atmega from bytes to ascii string
def motors(m1,m2,m3):
	motorValues = [m1,m2,m3]
	for x in range(3):
		ser.write(("m %d %d %d\r" % (x, abs(motorValues[x]), int(motorValues[x]>=0))).encode())


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
				#~ print(pwm)
				#~ print(data)			
				file.writelines(" {0} , {1}".format(data[:-2],rpm))
				#~ file.writelines(data)
		file.close		

		
		







