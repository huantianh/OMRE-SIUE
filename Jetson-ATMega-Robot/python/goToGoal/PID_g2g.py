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

#odemetry position
current_x = 0
current_y = 0
current_theta = 0

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
	return int(encoderValue.rstrip())

def ultrasound(ultraSoundNum):
	ser.reset_input_buffer()
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

####################################################################################################################
#This specific version of move will not allow RPM to go over a certain value, if you give it a command that causes
#RPM to go over it will decrease all motors by a ratio so that it fits in the bounds of motor RPM
####################################################################################################################
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
	
	maxAllowedSpeed = 150
	if (abs(wheel1RPM) > maxAllowedSpeed or abs(wheel0RPM) > maxAllowedSpeed or abs(wheel2RPM) > maxAllowedSpeed):
		maxRPM = max(abs(motor_spd_vec))
		ratio = abs(maxRPM)/maxAllowedSpeed
		wheel0RPM = wheel0RPM/ratio
		wheel1RPM = wheel1RPM/ratio
		wheel2RPM = wheel2RPM/ratio

	print("Wheel0 RPM: " +str(wheel0RPM))
	print("Wheel1 RPM: " +str(wheel1RPM))
	print("Wheel2 RPM: " +str(wheel2RPM))

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

#Sets up odemetry for when the program starts, we need this because arduino stores the encoders 
#and this will "Zero" it. 
def initOdometry():
	global oldEncoder0 
	global oldEncoder1 
	global oldEncoder2 
	oldEncoder0 = encoder(0)
	oldEncoder1 = encoder(1)
	oldEncoder2 = encoder(2)

def goToGoalPID(xd,yd,thetad):
	global current_x
	global current_y
	global current_theta
	dt = 0.1
	
	#PID goToGoal
	Kp = 2
	Ki = 0.05
	Kd = 0
	integral = np.array([0,0,0])[:,None]
	
	preError = np.array([0,0,0])[:,None]
	min_distance = 0.1
	
	delta = np.sqrt(((xd-current_x)**2)+((yd-current_y)**2))
	
	#~ start = time.time()
	
	#~ while time.time()-float(start) <= float(duration):
	while delta > min_distance:
		
		xc = current_x
		yc = current_y
		thetac = current_theta
		
		#PID Controller		
		setPoint = np.array([xd,yd,thetad])[:,None]
		currentPoint = np.array([xc,yc,thetac])[:,None]
		error = setPoint - currentPoint
		preError = error
		integral = integral + error
	
		#~ if error == 0:
			#~ integral = 0
		#~ if error >= 30:
			#~ integral = 0
	
		derivative = error - preError
		output = Kp*error + Ki*integral + Kd*derivative	
		vel_global = output
		#Inverse Kinematic 
		inv_rotation_mat= np.array([np.cos(thetac), np.sin(thetac), 0, -np.sin(thetac), np.cos(thetac), 0, 0, 0, 1]).reshape(3,3)
		#~ d = np.sqrt(((xd-xc)**2)+((yd-yc)**2))
		#~ phi = math.atan2((yd-yc),(xd-xc))
		
		#~ vel_global = np.array([ d*np.cos(phi), d*np.sin(phi), 0])[:,None]
		vel_local = np.dot(inv_rotation_mat, vel_global)		
		
		#~ time_left = duration - (time.time() - start) #duration - time elapsed = time left
		time_left = 1 #duration - time elapsed = time left
		vl_x = vel_local[0] / time_left
		vl_y = vel_local[1] / time_left 
		vl_theta = vel_local[2] / time_left
		
		#Move the robot
		move(vl_x,vl_y,vl_theta)
		#Odemetry
		pose = odemetryCalc(current_x,current_y,current_theta)
		current_x = pose.item(0)
		current_y = pose.item(1)
		current_theta = pose.item(2)
		
		delta = np.sqrt(((xd-current_x)**2)+((yd-current_y)**2))
				
		time.sleep(dt)
		data_write = "x: "+str(pose[0][0])+"  y: "+str(pose[1][0])+"  theta: "+str(pose[2][0])
		print(data_write)

	move(0,0,0)


while True: 
	print("######### Enter your goal (x,y) :) ########## ")
	xd = float(input("enter x desired: "))
	yd = float(input("enter y desired: "))
	thetad = float(input("enter theta desired: "))	
	#~ duration = float(input("enter duration desired: "))	
	goToGoalPID(xd,yd,thetad)


	
	
