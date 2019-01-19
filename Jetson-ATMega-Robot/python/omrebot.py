import serial
import math
import xbox
import time
import numpy as np

class OmreBot:
	def __init__(self,port):
		self.port = port
		self.ser = serial.Serial('/dev/ttyACM0',115200, timeout=.4);
		time.sleep(1)
		self.ser.reset_input_buffer()
		self.ser.reset_output_buffer()

	def printPort(self):
		print(self.port)


	def motors(m1,m2,m3):
		motorValues = [m1,m2,m3]
		for x in range(3):
			ser.write(("m %d %d %d\r" % (x, abs(motorValues[x]), int(motorValues[x]>=0))).encode())

	def motorVelocity(m1,m2,m3):
		motorV= [m1*10,m2*10,m3*10]
		ser.write(("v %d %d %d \r" %(motorV[0],motorV[1],motorV[2])).encode())

	#read encoder value from motor number given
	def encoder(encoderNum):
		ser.reset_input_buffer()
		ser.write(("e %d \r" % (encoderNum)).encode())
	
		encoderValue = (ser.readline().decode("ascii"))
		return encoderValue.rstrip()

	def ultrasound(ultraSoundNum):
		#ser.reset_input_buffer()
		ser.write(("u %d \r" % (ultraSoundNum)).encode())
		ultraSoundValue = (ser.readline().decode("ascii"))
		return int(ultraSoundValue.rstrip())

	def infrared(infraredNum):
		ser.reset_input_buffer()
		ser.write(("i %d \r" % (infraredNum)).encode())
		infraredValue = (ser.readline().decode("ascii"))
		return infraredValue.rstrip()


	def rpm(rpmNum):
		ser.reset_input_buffer()
		ser.write(("r %f \r" % (rpmNum)).encode())
		rpmValue = (ser.readline().decode("ascii"))
		return rpmValue.rstrip()

	def enablePID(pidValue):
		pid = pidValue
		ser.write(("p %d \r" % (pid)).encode())
