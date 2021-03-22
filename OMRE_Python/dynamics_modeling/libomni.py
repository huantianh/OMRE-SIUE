import serial
import math
import time
import numpy as np

# Serial setup, this lets you plug the arduino into any USB port on the raspi
i = 0
portname = '/dev/ttyACM'
while True:
    try:
        ser = serial.Serial(portname + str(i), 115200, timeout=1)
        ser.flushInput()
        ser.flushOutput()
        break
    except:
        i += 1
#wait for serial connection to be established
time.sleep(1)

#clear buffers just incase garbage inside
ser.reset_input_buffer()
ser.reset_output_buffer()


# This functions sends  pwm signals to the motor and reverses the direction if given negative
# Example motor(255,0,0) would turn motor 0 on all the away and 1,2 off
# motor(125,-200,-100) motor 0 would have a half duty cycle, motor 1 would move backwards at a pwm of 200 etc...
def motor_pwm(m1,m2,m3):
	motorValues = [m1,m2,m3]
	for x in range(3):
		ser.write(("m %d %d \r" % (x, motorValues[x])).encode())

def motor_rpm(m1,m2,m3):
	motorV= [m1,m2,m3]
	ser.write(("v %d %d %d \r" %(motorV[0],motorV[1],motorV[2])).encode())

#read encoder value from motor number given
def encoder(encoderNum):
	ser.reset_input_buffer()
	ser.write(("e %d \r" % (encoderNum)).encode())
	encoderValue = (ser.readline().decode("ascii"))
	return int(encoderValue.rstrip())

def ultrasonic(ultraSonicNum):
	ser.reset_input_buffer()
	ser.write(("u %d \r" % (ultraSonicNum)).encode())
	ultraSonicValue = (ser.readline().decode("ascii"))
	return float(ultraSonicValue.rstrip())

def infrared(infraredNum):
	ser.reset_input_buffer()
	ser.write(("i %d \r" % (infraredNum)).encode())
	infraredValue = (ser.readline().decode("ascii"))
	return infraredValue.rstrip()

def rpm(rpmNum):
	ser.reset_input_buffer()
	ser.write(("r %f \r" % (rpmNum)).encode())
	rpmValue = (ser.readline().decode("ascii"))
	return float(rpmValue.rstrip())
	
def motor_current(curNum):
	ser.reset_input_buffer()
	ser.write(("c %f \r" % (curNum)).encode())
	curValue = (ser.readline().decode("ascii"))
	return float(curValue.rstrip())	
	
def motor_voltage(volNum):
	ser.reset_input_buffer()
	ser.write(("t %f \r" % (volNum)).encode())
	volValue = (ser.readline().decode("ascii"))
	return float(volValue.rstrip())		

def stop():
	ser.write(("s \r").encode())	

def move(v_x, v_y, v_theta):

	r = 0.03 # radius of each wheel [m]
	l = 0.19 # distance from each wheel to the point of reference [m]
 
	xd_des = v_x # velocity in the x-direction in the local frame [m/s]
	yd_des = v_y # velocity in the y-direction in the local frame [m/s]
	thd_des = v_theta # velocity in the x-direction in the local frame [rad/sa]

	vel_des = np.array([xd_des,yd_des,thd_des]).reshape(3,1)
 
	FK_M = (2*np.pi*r/60)*np.array([1/np.sqrt(3),0,-1/np.sqrt(3),-1/3,2/3,-1/3,-1/(3*l),-1/(3*l),-1/(3*l)]).reshape(3,3) # Forward kinematics matrix
 
	IK_M = np.linalg.inv(FK_M) # Inverse kinematics matrix
 
	motor_spd_vec = np.dot(IK_M,vel_des, out=None)
 
	wheel1RPM = motor_spd_vec[0] # motor 2 speed [rpm]
	wheel0RPM = motor_spd_vec[1] # motor 1 speed [rpm]
	wheel2RPM = motor_spd_vec[2] # motor 3 speed [rpm]
	
	maxAllowedSpeed = 150
	
	if (abs(wheel1RPM) > maxAllowedSpeed or abs(wheel0RPM) > maxAllowedSpeed or abs(wheel2RPM) > maxAllowedSpeed):
		maxRPM = max(abs(motor_spd_vec))
		ratio = abs(maxRPM)/maxAllowedSpeed
		
		wheel0RPM = wheel0RPM/ratio
		wheel1RPM = wheel1RPM/ratio
		wheel2RPM = wheel2RPM/ratio
	
	# ~ #~ print("Wheel0 RPM: " +str(wheel0RPM))
	# ~ #~ print("Wheel1 RPM: " +str(wheel1RPM))
	# ~ #~ print("Wheel2 RPM: " +str(wheel2RPM))

	motor_rpm(int(wheel0RPM),int(wheel1RPM),int(wheel2RPM))
