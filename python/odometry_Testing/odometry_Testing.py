import serial
import math
import xbox
import time
import numpy as np
import libomni
import os


ser = serial.Serial('/dev/ttyACM0',9600, timeout=.4);
time.sleep(1)
ser.reset_input_buffer()
ser.reset_output_buffer()


#folder where saving all the data
save_folder = "Odometry_Test_Data/"
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

def PIValues(Kp,Ki):
	ser.reset_input_buffer()
	ser.write(("k %s %s \r" % (Kp,Ki)).encode())


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
	
	#for x in range(3):
	#	time.sleep(.5)
	#	ser.write(('v %d %d \n \r' % (x,motorValues[x])).encode()) #turning into int should fix !!!!!!!!!!!!!!!!!!
	#print("Finished all commands")

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

	if (abs(wheel1RPM) > 200 or abs(wheel0RPM) > 200 or abs(wheel2RPM) > 200):
		if  (abs(wheel0RPM) > abs(wheel1RPM) and abs(wheel0RPM) > abs(wheel2RPM)):
			ratio = abs(wheel0RPM)/200
			wheel0RPM = wheel0RPM/ratio
			wheel1RPM = wheel1RPM/ratio
			wheel2RPM = wheel2RPM/ratio
		elif (abs(wheel1RPM) > abs(wheel0RPM) and abs(wheel1RPM) > abs(wheel2RPM)):
			ratio = abs(wheel1RPM)/200
			wheel0RPM = wheel0RPM/ratio
			wheel1RPM = wheel1RPM/ratio
			wheel2RPM = wheel2RPM/ratio
		else:
			ratio = abs(wheel2RPM)/200
			wheel0RPM = wheel0RPM/ratio
			wheel1RPM = wheel1RPM/ratio
			wheel2RPM = wheel2RPM/ratio

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



#velocityValues(0,0,0)
#xyThetaToWheelV(0,0,0)
#readEncoders()


mode = str(input("Enter mode. s for serial, t for input tester, c for controller, g for graph mode "))

if(mode == 's'):

############## Simple Serial Communicator to Arduino ##############
	while True:
		command = input("Enter Command: ")
		command = command+'\r'
		ser.write(command.encode())
		print (ser.readline().decode("ascii"))
		
elif mode == 't':
################ Simple Input Tester Loop ###############
	while True:
		yesNo = input("do you want to quit y/n")
		if(yesNo == 'y'):
			xyThetaToWheelV(0,0,0)
			ser.close()
			quit()
		time.sleep(.5)
		x = float(input("enter x: "))
		y = float(input("enter y: "))
		theta = float(input("enter theta: "))
		mytime  = float(input("enter time to run: "))
		xyThetaToWheelV(x,y,theta)
		time.sleep(mytime)
		xyThetaToWheelV(0,0,0)


elif mode == 'o':
	while True:
		timer = input("Enter time to run: ")
		x = float(input("enter x: "))
		y = float(input("enter y: "))
		theta = float(input("enter theta: "))
		start = time.time()
		xyThetaToWheelV(x,y,theta)
		oldEncoder0 = readEncoder(0)
		oldEncoder1 = readEncoder(1)
		oldEncoder2 = readEncoder(2)
	
		old_x = 0
		old_y = 0
		old_t = 0

		file = open(save_folder + "data_x_"+str(x)+",y_"+str(y)+",theta_"+str(theta)+",time_"+str(timer)+".txt","w+")

		while True:
			newEncoder0 = readEncoder(0)
			newEncoder1 = readEncoder(1)
			newEncoder2 = readEncoder(2)
			deltaEncoder0 = newEncoder0 - oldEncoder0
			deltaEncoder1 = newEncoder1 - oldEncoder1
			deltaEncoder2 = newEncoder2 - oldEncoder2
			Ds=DValue(deltaEncoder0,deltaEncoder1,deltaEncoder2)
			D0 = Ds.item(0)
			D1 = Ds.item(1)
			D2 = Ds.item(2)
		
			pose = odemetryCalc(old_x,old_y,old_t,D0,D1,D2)

#write data
			data_write = "x: "+str(pose[0][0])+"  y: "+str(pose[1][0])+"  theta: "+str(pose[2][0]);	
			print(data_write)
			file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+"\n")
			
			old_x=pose.item(0)
			old_y=pose.item(1)
			old_t=pose.item(2)
			oldEncoder0 = newEncoder0
			oldEncoder1 = newEncoder1
			oldEncoder2 = newEncoder2
			if time.time()-float(start) >= float(timer):
				xyThetaToWheelV(0,0,0)
				file.close()
				break
				
elif mode == 'p':
	while True:
		p = input("enter Kp: ")
		i = input("enter Ki: ")
		x = float(input("enter x: "))
		y = float(input("enter y: "))
		theta = float(input("enter theta: "))
		PIValues(p,i)
		xyThetaToWheelV(x,y,theta)
		







