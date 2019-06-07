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

pid = 0;

# This functions sends  pwm signals to the motor and reverses the direction if given negative
# Example motor(255,0,0) would turn motor 0 on all the away and 1,2 off
# motor(125,-200,-100) motor 0 would have a half duty cycle, motor 1 would move backwards at a pwm of 200 etc...
def motors(m1,m2,m3):
	motorValues = [m1,m2,m3]
	for x in range(3):
		ser.write(("m %d %d %d\r" % (x, abs(motorValues[x]), int(motorValues[x]>=0))).encode())

def motorVelocity(m1,m2,m3):
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
	return rpmValue.rstrip()

def enablePID(pidValue):
	pid = pidValue
	ser.write(("p %d \r" % (pid)).encode())
	
def stop():
	ser.write(("s \r").encode())	

def move(xd,yd,thetad):

	r = 0.03 # radius of each wheel [m]
	l = 0.19 # distance from each wheel to the point of reference [m]
 
	xd_des = xd # velocity in the x-direction in the local frame [m/s]
	yd_des = yd # velocity in the y-direction in the local frame [m/s]
	thd_des = thetad # velocity in the x-direction in the local frame [rad/sa]
 
	vel_des = np.array([xd_des,yd_des,thd_des]).reshape(3,1)
 
	FK_M = (2*np.pi*r/60)*np.array([1/np.sqrt(3),0,-1/np.sqrt(3),-1/3,2/3,-1/3,-1/(3*l),-1/(3*l),-1/(3*l)]).reshape(3,3) # Forward kinematics matrix
 
	IK_M = np.linalg.inv(FK_M) # Inverse kinematics matrix
 
	motor_spd_vec = np.dot(IK_M,vel_des, out=None)
 
	wheel1RPM = motor_spd_vec[0] # motor 2 speed [rpm]
	wheel0RPM = motor_spd_vec[1] # motor 1 speed [rpm]
	wheel2RPM = motor_spd_vec[2] # motor 3 speed [rpm]
	
	maxAllowedSpeed = 100
	
	if (abs(wheel1RPM) > maxAllowedSpeed or abs(wheel0RPM) > maxAllowedSpeed or abs(wheel2RPM) > maxAllowedSpeed):
		maxRPM = max(abs(motor_spd_vec))
		ratio = abs(maxRPM)/maxAllowedSpeed
		
		wheel0RPM = wheel0RPM/ratio
		wheel1RPM = wheel1RPM/ratio
		wheel2RPM = wheel2RPM/ratio
	
	#~ print("Wheel0 RPM: " +str(wheel0RPM))
	#~ print("Wheel1 RPM: " +str(wheel1RPM))
	#~ print("Wheel2 RPM: " +str(wheel2RPM))

	motorVelocity(int(wheel0RPM),int(wheel1RPM),int(wheel2RPM))

#~ def odemetryCalc(xk,yk,thetak,l=0.19, N=2249, r=0.03):
	#~ global oldEncoder0
	#~ global oldEncoder1
	#~ global oldEncoder2
	
	#~ newEncoder0 = encoder(0)
	#~ newEncoder1 = encoder(1)
	#~ newEncoder2 = encoder(2)
	
	#~ deltaEncoder0 = newEncoder0 - oldEncoder0
	#~ deltaEncoder1 = newEncoder1 - oldEncoder1
	#~ deltaEncoder2 = newEncoder2 - oldEncoder2
	
	#~ D0=(deltaEncoder0/N)*((2*np.pi*r))
	#~ D1=(deltaEncoder1/N)*((2*np.pi*r))
	#~ D2=(deltaEncoder2/N)*((2*np.pi*r))

	#~ kinematic_mat = np.array([1/np.sqrt(3),0,-1/np.sqrt(3),-1/3,2/3,-1/3,-1/(3*l),-1/(3*l),-1/(3*l)]).reshape(3,3)
		
	#~ rotation_mat= np.array([np.cos(thetak),-np.sin(thetak),0,np.sin(thetak),np.cos(thetak),0,0,0,1]).reshape(3,3)
	
	#~ # diffrence in ticks (rpm)
	#~ distance_mat = np.array([D1,D0,D2])[:,None]
	
	#~ oldPos_mat = np.array([xk,yk,thetak])[:,None]
	
	#~ # np.dot explanation https://stackoverflow.com/questions/21562986/numpy-matrix-vector-multiplication
	#~ kinxrot = np.dot(rotation_mat,kinematic_mat)
	#~ newPos_mat = oldPos_mat + np.dot(kinxrot,distance_mat)

	#~ oldEncoder0 = newEncoder0
	#~ oldEncoder1 = newEncoder1
	#~ oldEncoder2 = newEncoder2

	#~ return  newPos_mat


