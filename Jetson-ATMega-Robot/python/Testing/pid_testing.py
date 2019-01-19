import serial
import math, time, os, libomni
import numpy as np


ser = serial.Serial('/dev/ttyACM0',115200, timeout=.4);


ser.reset_input_buffer()
ser.reset_output_buffer()
#joy = xbox.Joystick()

#folder where saving all the data
save_folder = "Data_Testing/PID_Testing/"
current_directory = os.getcwd()
save_directory = os.path.join(current_directory, save_folder)

def readEncoder(encoderNum):
	ser.reset_input_buffer()
	ser.write(("e %d \r" % (encoderNum)).encode())
	
	encoderValue = (ser.readline().decode("ascii"))
	#print(encoderValue)
	data = float(encoderValue.strip())
	#print(data)
	return data

def PIDValues(Kp,Ki,Kd):
	ser.reset_input_buffer()
	ser.write(("k %s %s %s \r" % (Kp,Ki,Kd)).encode())
	
def PIValues_3(Kp1,Kp2,Kp3,Ki1,Ki2,Ki3):
	ser.reset_input_buffer()
	ser.write(("w %s %s %s %s %s %s \r" % (Kp1,Kp2,Kp3,Ki1,Ki2,Ki3)).encode())

def enablePID(pidValue):
	pid = pidValue
	ser.write(("p %d \r" % (pid)).encode())
	
# Fun function that takes all 3 motor PWM value from -255  to 255 and interprets it 
# into the correct command to send to arduino. Remember with Python3  Pyserial pretty much
# expects everything  in forms of bytes so we encode it into a byte and decode the output from
# atmega from bytes to ascii string
def motors(m1,m2,m3):
	motorValues = [m1,m2,m3]
	for x in range(3):
		ser.write(("m %d %d %d\r" % (x, abs(motorValues[x]), int(motorValues[x]>=0))).encode())
		#print(ser.readline().decode("ascii"))
	#readEncoders()
def velocityValues(m1,m2,m3):
	motorValues = [m1,m2,m3]
	print(motorValues)
	ser.write(('v %d %d %d \r'  % (motorValues[0],motorValues[1],motorValues[2])).encode())
	
theta = 0
def velocityToPWM(velocity):
	theta = 0
	isNegative = False
	if(velocity < 0):
		velocity = abs(velocity)
		isNegative = True
	if(velocity == 0):
		return 0
	if(velocity > 0 and velocity < .36):
		if(isNegative):
			return -58
		return 58
	if(velocity >= .656):
		if(isNegative):
			return -252
		return 252

	PWMvalue = 0
	PWMValue = 124220*velocity**4 - 238623 * velocity**3 + 171697* velocity**2 - 54490* velocity + 6470
	if(isNegative):
		return PWMValue*-1
	return PWMValue


def xyThetaToWheelV(xd,yd,thetad):

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


	print("Wheel0 RPM: " +str(wheel0RPM))
	print("Wheel1 RPM: " +str(wheel1RPM))
	print("Wheel2 RPM: " +str(wheel2RPM))
	wheel0RPM *= 10
	wheel1RPM *= 10
	wheel2RPM *= 10
	

	velocityValues(int(wheel0RPM),int(wheel1RPM),int(wheel2RPM))
	#motors(100,100,100)


def odemetryCalc(xk,yk,thetak,D0,D1,D2,l=0.19):


	kinematic_mat = np.array([1/np.sqrt(3),0,-1/np.sqrt(3),-1/3,2/3,-1/3,-1/(3*l),-1/(3*l),-1/(3*l)]).reshape(3,3)
		
	rotation_mat= np.array([np.cos(thetak),-np.sin(thetak),0,np.sin(thetak),np.cos(thetak),0,0,0,1]).reshape(3,3)
	#   diffrence in ticks (rpm1)
	
	distance_mat = np.array([D1,D0,D2])[:,None]
	oldPos_mat = np.array([xk,yk,thetak])[:,None]
	
	# np.dot explanation https://stackoverflow.com/questions/21562986/numpy-matrix-vector-multiplication
	kinxrot = np.dot(rotation_mat,kinematic_mat)
	newPos_mat = oldPos_mat + np.dot(kinxrot,distance_mat)
	return  newPos_mat


def DValue(deltaEncoder0,deltaEncoder1,deltaEncoder2, r=0.03, N=2249):
	D0=(deltaEncoder0/N)*((2*np.pi*r))
	D1=(deltaEncoder1/N)*((2*np.pi*r))
	D2=(deltaEncoder2/N)*((2*np.pi*r))
	return np.array([D0,D1,D2])

mode = str(input("Enter mode: s for serial, g for 1 gain, k for 3 gains "))

if(mode == 's'):

############## Simple Serial Communicator to Arduino ##############
	while True:
		command = input("Enter Command: ")
		command = command+'\r'
		ser.write(command.encode())
		print (ser.readline().decode("ascii"))

###################### 1 GAIN  ##############################
elif mode == 'g':
	pid = 1	
	enablePID(pid)
	while True:
		timer = input("Enter time to run: ")
		p = input("enter Kp: ")
		i = input("enter Ki: ")
		d = input("enter Kd: ")
		
		PIDValues(p,i,d)
		command = input("Enter Command: ")
		command = command+'\r'
		ser.write(command.encode())
		file = open(save_folder + "Kp_"+p+",Ki_"+i+",Kd_"+d+".txt","w+")		
		start = time.time()
		while True:
			data = [0]
			if(ser.inWaiting() > 0):				
				data = (ser.readline().decode("ascii"))
				print (data)
				file.writelines(data)

			if time.time()-float(start) >= float(timer):
				xyThetaToWheelV(0,0,0)
				file.close()
				break

######################  3 GAINS   ############################
elif mode == 'k':
	pid = 1	
	enablePID(pid)
	while True:
		timer = input("Enter time to run: ")
		p1 = input("enter Kp 1: ")
		p2 = input("enter Kp 2: ")
		p3 = input("enter Kp 3: ")
		i1 = input("enter Ki 1: ")
		i2 = input("enter Ki 2: ")
		i3 = input("enter Ki 3: ")
		
		PIValues_3(p1,p2,p3,i1,i2,i3)
		command = input("Enter Command: ")
		command = command+'\r'
		ser.write(command.encode())
		file = open(save_folder +"kp1: "+p1+", kp2: "+p2+", kp3: "+p3+", ki1: "+i1+", ki2: "+i2+", ki3: "+i3+".txt","w+")		
		start = time.time()
		while True:
			data = [0]
			if(ser.inWaiting() > 0):				
				data = (ser.readline().decode("ascii"))
				print (data)
				file.writelines(data)

			if time.time()-float(start) >= float(timer):
				xyThetaToWheelV(0,0,0)
				file.close()
				break

				






