import serial
import math
import time
import numpy as np

#sets up serial connection to arduino atMega
ser = serial.Serial('/dev/ttyACM0',115200, timeout=.4);



#clear buffers just incase garbage inside
ser.reset_input_buffer()
ser.reset_output_buffer()

pid = 1;
oldEncoder0 = 0
oldEncoder1 = 0
oldEncoder2 = 0
newEncoder0 = 0
newEncoder1 = 0
newEncoder2 = 0
current_x = 0
current_y = 0
current_t = 0
# This functions sends  pwm signals to the motor and reverses the direction if given negative
# Example motor(255,0,0) would turn motor 0 on all the away and 1,2 off
# motor(125,-200,-100) motor 0 would have a half duty cycle, motor 1 would move backwards at a pwm of 200 etc...
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
	return float(encoderValue.rstrip())

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

def move(xd,yd,thetad):

	r = 0.03 # radius of each wheel [m]
	l = 0.19 # distance from each wheel to the point of reference [m]
 
	xd_des = xd # velocity in the x-direction in the local frame [m/s]
	yd_des = yd # velocity in the y-direction in the local frame [m/s]
	thd_des = thetad # velocity in the x-direction in the local frame [rad/sa]
 
	vel_des = np.array([xd_des,yd_des,thd_des])[:,None]
 
	FK_M = (2*np.pi*r/60)*np.array([1/np.sqrt(3),0,-1/np.sqrt(3),-1/3,2/3,-1/3,-1/(3*l),-1/(3*l),-1/(3*l)]).reshape(3,3) # Forward kinematics matrix
 
	IK_M = np.linalg.inv(FK_M) # Inverse kinematics matrix
 
	motor_spd_vec = np.dot(IK_M,vel_des)
 
	wheel1RPM = motor_spd_vec[0] # motor 2 speed [rpm]
	wheel0RPM = motor_spd_vec[1] # motor 1 speed [rpm]
	wheel2RPM = motor_spd_vec[2] # motor 3 speed [rpm]


	#print("Wheel0 RPM: " +str(wheel0RPM))
	#print("Wheel1 RPM: " +str(wheel1RPM))
	#print("Wheel2 RPM: " +str(wheel2RPM))
	#wheel0RPM *= 10
	#wheel1RPM *= 10
	#wheel2RPM *= 10
	

	motorVelocity(int(wheel0RPM),int(wheel1RPM),int(wheel2RPM))



def odemetryCalc(xk,yk,thetak,l=0.19, N=2249, r=0.03):
	global oldEncoder0
	global oldEncoder1
	global oldEncoder2
	
	newEncoder0 = encoder(0)
	newEncoder1 = encoder(1)
	newEncoder2 = encoder(2)
	
	deltaEncoder0 = newEncoder0 - oldEncoder0
	deltaEncoder1 = newEncoder1 - oldEncoder1
	deltaEncoder2 = newEncoder2 - oldEncoder2
	
	D0=(deltaEncoder0/N)*((2*np.pi*r))
	D1=(deltaEncoder1/N)*((2*np.pi*r))
	D2=(deltaEncoder2/N)*((2*np.pi*r))

	kinematic_mat = np.array([1/np.sqrt(3),0,-1/np.sqrt(3),-1/3,2/3,-1/3,-1/(3*l),-1/(3*l),-1/(3*l)]).reshape(3,3)
		
	rotation_mat= np.array([np.cos(thetak),-np.sin(thetak),0,np.sin(thetak),np.cos(thetak),0,0,0,1]).reshape(3,3)
	
	#   diffrence in ticks (rpm1)
	distance_mat = np.array([D1,D0,D2])[:,None]
	
	oldPos_mat = np.array([xk,yk,thetak])[:,None]
	
	# np.dot explanation https://stackoverflow.com/questions/21562986/numpy-matrix-vector-multiplication
	kinxrot = np.dot(rotation_mat,kinematic_mat)
	newPos_mat = oldPos_mat + np.dot(kinxrot,distance_mat)

	oldEncoder0 = newEncoder0
	oldEncoder1 = newEncoder1
	oldEncoder2 = newEncoder2

	return  newPos_mat
	



#might not work if negative and velocity is 0
def rotate(radians,velocity,timer):
	
	if velocity == 0:
		velocity = radians/timer
		print("velocity calculated: " +str(velocity))
		move(0,0,velocity)
		return velocity;
	else:
		if radians <0:
			velocity = velocity*-1
			timer = abs(radians/velocity)
		print("time calculated: " +str(timer))
		move(0,0,velocity)
		return timer;
		
def moveX(distance,velocity,timer):
	if velocity == 0:
		velocity = distance/timer
		print("velocity calculated: " +str(velocity))
		move(velocity,0,0)
		return;
	else:
		timer = abs(distance/velocity)
		print("time calculated: " +str(timer))
		move(velocity,0,0)
		return timer
		
		
def distForm(cx,cy,dx,dy):
	return np.sqrt(((dx-cx)**2)+((dy-cy)**2))
	
	
def currentGlobalFrame(currentLocalFrame):

	currentLocalFrame = (currentLocalFrame-2*np.pi) % (2*np.pi)
	return currentLocalFrame

while True:

	print("######### Enter your goal (x,y) :) ########## ")
	dx = float(input("enter d_x: "))
	dy = float(input("enter d_y: "))
	
	#rotate to global frame
	print("current x = "+str(current_x))
	print("current y = "+str(current_y))
	
	#rotationAngle = math.atan2((dy-current_y),(dx-current_x))
	atan2 = math.atan2((dy-current_y),(dx-current_x))
	print("Atan2 rotation = "+str(atan2))
	
	
	#atan2 returns a number between 180 to -180. We want to turn this into a global angle (0 - 2pi)
	atan2Global = currentGlobalFrame(atan2)
	print("Global frame of atan2 calculation = "+str(atan2Global))
	print("Current theta = "+str(current_t) + " global = "+ str(currentGlobalFrame(current_t)))
	
	rotation = (atan2Global - currentGlobalFrame(current_t)) #% (2*np.pi))
	print ("rotation in radians from current to destination rotation = "+str(rotation))
	
	timer = rotate(rotation,2,0)
	start = time.time()

	while True:
				
		pose = odemetryCalc(current_x,current_y,current_t)

		current_x = pose.item(0)
		current_y = pose.item(1)
		current_t = pose.item(2)

		if time.time()-float(start) >= float(timer):
			move(0,0,0)
			break
			
	
	print("---------------------------------")
	distance = distForm(current_x,current_y,dx,dy)
	print(" distance to travel: "+str(distance))
	timer = moveX(distance,.4,0)
	start = time.time()
	while True:
		pose = odemetryCalc(current_x,current_y,current_t)

		#data_write = "x: "+str(pose[0][0])+"  y: "+str(pose[1][0])+"  theta: "+str(pose[2][0])
		#print(data_write)
		#file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+"\n")

		current_x = pose.item(0)
		current_y = pose.item(1)
		current_t = pose.item(2)

		if time.time()-float(start) >= float(timer):
			data_write = "x: "+str(pose[0][0])+"  y: "+str(pose[1][0])+"  theta: "+str(pose[2][0])
			print(data_write)
			move(0,0,0)
			break
		
				

