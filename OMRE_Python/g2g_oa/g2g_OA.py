import libomni as robot  #Library tha handles all the serial commands to arduino AtMega
import time
import serial
import math
import numpy as np


#ultrasonic setup
us = [0,0,0,0,0,0]
d  = [0,0,0,0,0,0]
#odemetry setup
oldEncoder0 = 0
oldEncoder1 = 0
oldEncoder2 = 0
newEncoder0 = 0
newEncoder1 = 0
newEncoder2 = 0


####################################################		Reset encoder
def initOdometry():
	global oldEncoder0 
	global oldEncoder1 
	global oldEncoder2 
	oldEncoder0 = robot.encoder(0)
	oldEncoder1 = robot.encoder(1)
	oldEncoder2 = robot.encoder(2)
	
#odemetry position
current_x = 0
current_y = 0
current_theta = 0

vl_s = [0,0,0,1]
flag = [0,0,0,0,0,0]
v_x = 0
v_y = 0
v_theta = 0

#####################################################		Obstacle Position
def us0(d0):
	#~ T_mat_robot_world= np.array([np.cos(thetac),-np.sin(thetac),0,xc,np.sin(thetac),np.cos(thetac),0,yc,0,0,1,0,0,0,0,1]).reshape(4,4)
	T_mat_sen0_robot = np.array([-1,0,0,-0.33,0,-1,0,0,0,0,1,0,0,0,0,1]).reshape(4,4)
	P_obs0_sen0		 = np.array([d0,0,0,1]).reshape(4,1)	
	P_obs0_robot 	 = np.dot(T_mat_sen0_robot,P_obs0_sen0)
	#~ P_obs0_world	 = np.dot(T_mat_robot_world,P_obs0_robot)	
	#~ P_obs0			 = np.concatenate([P_obs0_robot,P_obs0_world])
	return 	P_obs0_robot
	
def us1(d1):
	#~ T_mat_robot_world= np.array([np.cos(thetac),-np.sin(thetac),0,xc,np.sin(thetac),np.cos(thetac),0,yc,0,0,1,0,0,0,0,1]).reshape(4,4)
	T_mat_sen1_robot = np.array([0,-1,0,0,1,0,0,0.33,0,0,1,0,0,0,0,1]).reshape(4,4)
	P_obs1_sen1		 = np.array([d1,0,0,1]).reshape(4,1)	
	P_obs1_robot 	 = np.dot(T_mat_sen1_robot,P_obs1_sen1)
	#~ P_obs1_world	 = np.dot(T_mat_robot_world,P_obs1_robot)	
	#~ P_obs1			 = np.concatenate([P_obs1_robot,P_obs1_world])
	return 	P_obs1_robot 
	
def us2(d2):
	#~ T_mat_robot_world= np.array([np.cos(thetac),-np.sin(thetac),0,xc,np.sin(thetac),np.cos(thetac),0,yc,0,0,1,0,0,0,0,1]).reshape(4,4)
	T_mat_sen2_robot = np.array([np.sqrt(3)/2,-1/2,0,0.3556*(np.sqrt(3)/2),1/2,np.sqrt(3)/2,0,0.3556/2,0,0,1,0,0,0,0,1]).reshape(4,4)
	P_obs2_sen2		 = np.array([d2,0,0,1]).reshape(4,1)	
	P_obs2_robot 	 = np.dot(T_mat_sen2_robot,P_obs2_sen2)
	#~ P_obs2_world	 = np.dot(T_mat_robot_world,P_obs2_robot)
	#~ P_obs2			 = np.concatenate([P_obs2_robot,P_obs2_world])
	return 	P_obs2_robot 

def us3(d3):
	#~ T_mat_robot_world= np.array([np.cos(thetac),-np.sin(thetac),0,xc,np.sin(thetac),np.cos(thetac),0,yc,0,0,1,0,0,0,0,1]).reshape(4,4)
	T_mat_sen3_robot = np.array([1,0,0,0.33,0,1,0,0,0,0,1,0,0,0,0,1]).reshape(4,4)
	P_obs3_sen3		 = np.array([d3,0,0,1]).reshape(4,1)	
	P_obs3_robot 	 = np.dot(T_mat_sen3_robot,P_obs3_sen3)
	#~ P_obs3_world	 = np.dot(T_mat_robot_world,P_obs3_robot)	
	#~ P_obs3			 = np.concatenate([P_obs3_robot,P_obs3_world])
	return 	P_obs3_robot 

def us4(d4):
	#~ T_mat_robot_world= np.array([np.cos(thetac),-np.sin(thetac),0,xc,np.sin(thetac),np.cos(thetac),0,yc,0,0,1,0,0,0,0,1]).reshape(4,4)
	T_mat_sen4_robot = np.array([np.sqrt(3)/2,1/2,0,0.3556*(np.sqrt(3)/2),-1/2,np.sqrt(3)/2,0,-0.3556/2,0,0,1,0,0,0,0,1]).reshape(4,4)
	P_obs4_sen4		 = np.array([d4,0,0,1]).reshape(4,1)	
	P_obs4_robot 	 = np.dot(T_mat_sen4_robot,P_obs4_sen4)
	#~ P_obs4_world	 = np.dot(T_mat_robot_world,P_obs4_robot)	
	#~ P_obs4			 = np.concatenate([P_obs4_robot,P_obs4_world])
	return 	P_obs4_robot 

def us5(d5):
	#~ T_mat_robot_world=  np.array([np.cos(thetac),-np.sin(thetac),0,xc,np.sin(thetac),np.cos(thetac),0,yc,0,0,1,0,0,0,0,1]).reshape(4,4)
	T_mat_sen5_robot = np.array([0,1,0,0,-1,0,0,-0.33,0,0,1,0,0,0,0,1]).reshape(4,4)
	P_obs5_sen5		 = np.array([d5,0,0,1]).reshape(4,1)	
	P_obs5_robot 	 = np.dot(T_mat_sen5_robot,P_obs5_sen5)
	#~ P_obs5_world	 = np.dot(T_mat_robot_world,P_obs5_robot)
	#~ P_obs5			 = np.concatenate([P_obs5_robot,P_obs5_world])
	return 	P_obs5_robot 

######################################################		Odemetry	
def odemetryCalc(xk,yk,thetak,l=0.19, N=2249, r=0.03):
	global oldEncoder0
	global oldEncoder1
	global oldEncoder2
	
	newEncoder0 = robot.encoder(0)
	newEncoder1 = robot.encoder(1)
	newEncoder2 = robot.encoder(2)
	
	deltaEncoder0 = newEncoder0 - oldEncoder0
	deltaEncoder1 = newEncoder1 - oldEncoder1
	deltaEncoder2 = newEncoder2 - oldEncoder2
	
	D0=(deltaEncoder0/N)*((2*np.pi*r))
	D1=(deltaEncoder1/N)*((2*np.pi*r))
	D2=(deltaEncoder2/N)*((2*np.pi*r))

	kinematic_mat = np.array([1/np.sqrt(3),0,-1/np.sqrt(3),-1/3,2/3,-1/3,-1/(3*l),-1/(3*l),-1/(3*l)]).reshape(3,3)
		
	rotation_mat= np.array([np.cos(thetak),-np.sin(thetak),0,np.sin(thetak),np.cos(thetak),0,0,0,1]).reshape(3,3)
	
	# diffrence in ticks (rpm)
	distance_mat = np.array([D1,D0,D2])[:,None]
	
	oldPos_mat = np.array([xk,yk,thetak])[:,None]
	
	# np.dot explanation https://stackoverflow.com/questions/21562986/numpy-matrix-vector-multiplication
	kinxrot = np.dot(rotation_mat,kinematic_mat)
	newPos_mat = oldPos_mat + np.dot(kinxrot,distance_mat)

	oldEncoder0 = newEncoder0
	oldEncoder1 = newEncoder1
	oldEncoder2 = newEncoder2

	return  newPos_mat


#######################################################		G2G_OA_Blend
def g2g_oa(xd,yd,thetad):

#########################################			G2G					#####################################
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
	min_distance = 0.15
	
	delta = np.sqrt(((xd-current_x)**2)+((yd-current_y)**2))
	
	while delta > min_distance:
		
		xc = current_x
		yc = current_y
		thetac = current_theta
		
		pose = odemetryCalc(xc,yc,thetac)
		
		#PID Controller		
		setPoint = np.array([xd,yd,thetad])[:,None]
		currentPoint = np.array([xc,yc,thetac])[:,None]
		error = setPoint - currentPoint
		preError = error
		integral = integral + error
	
		derivative = error - preError
		output = Kp*error + Ki*integral + Kd*derivative	
		vel_global = output
		
		#Inverse Kinematic 
		inv_rotation_mat= np.array([np.cos(thetac), np.sin(thetac), 0, -np.sin(thetac), np.cos(thetac), 0, 0, 0, 1]).reshape(3,3)
		
		#~ vel_global = np.array([ d*np.cos(phi), d*np.sin(phi), 0])[:,None]
		vel_local = np.dot(inv_rotation_mat, vel_global)		
		
		v_x_g2g = vel_local[0]
		v_y_g2g = vel_local[1] 
		v_theta_g2g = vel_local[2] 
		
		
################################################################################################################
#########################################		Obstacle Avoidance					############################
		for x in range(6):
			us[x] = robot.ultrasonic(x)
			#~ print(str(us[0])+", "+str(us[1])+", "+str(us[2])+", "+str(us[3])+", "+str(us[4])+", "+str(us[5]))
			
			global d
			d = [us[0],us[1],us[2],us[3],us[4],us[5]]

			if d[x] != 0:
				flag[x] = 1
			else:
				flag[x] = 0	
			#~ print(flag)
		
			if d[x] != 0:
				
				d0 = d[0]
				d1 = d[1]
				d2 = d[2]
				d3 = d[3]
				d4 = d[4]
				d5 = d[5]
							
				vs0 = us0(d0)
				vs1 = us1(d1)
				vs2 = us2(d2)
				vs3 = us3(d3)
				vs4 = us4(d4)
				vs5 = us5(d5)	
				
				global vl_s
							
				vl_s = flag[0]*vs0 + flag[1]*vs1 + flag[2]*vs2 + flag[3]*vs3 + flag[4]*vs4 + flag[5]*vs5			
			
###############################################			Combine G2G and OA  		#####################################
	
		v_x_obs = -vl_s[0]
		v_y_obs = -vl_s[1]	
		
		#~ print(str(v_x_obs)+' , '+str(v_y_obs))
		#~ print(d)
		for x in d:
			global v_x
			global v_y
			global v_theta	
			if d == [0,0,0,0,0,0]:	
	
				v_x = 0.3*v_x_g2g 
				v_y = 0.3*v_y_g2g 
				v_theta = v_theta_g2g
			
			elif x > 0.3 and x <= 0.4:
				
				v_x = 0.3*v_x_g2g + 0.7*v_x_obs
				v_y = 0.3*v_y_g2g + 0.7*v_y_obs
				v_theta = v_theta_g2g
				#~ print('30 cm')	
			
			elif x > 0.2 and x <= 0.3:
				
				v_x = 0.2*v_x_g2g + 0.8*v_x_obs
				v_y = 0.2*v_y_g2g + 0.8*v_y_obs
				v_theta = v_theta_g2g
				#~ print('20 cm')		
			
			elif x > 0.1 and x <= 0.2:
				
				v_x = 0.1*v_x_g2g + 0.9*v_x_obs
				v_y = 0.1*v_y_g2g + 0.9*v_y_obs
				v_theta = v_theta_g2g
				#~ print('10 cm')		
			
			elif x > 0 and x <= 0.1:
				
				v_x = v_x_obs
				v_y = v_y_obs
				v_theta = v_theta_g2g
				#~ print('0 cm')	
		
		robot.move(v_x,v_y,v_theta)	
		#Odemetry
		current_x = pose.item(0)
		current_y = pose.item(1)
		current_theta = pose.item(2)

		delta = np.sqrt(((xd-current_x)**2)+((yd-current_y)**2))
				
		time.sleep(dt)
		data_write = "x: "+str(pose[0][0])+"  y: "+str(pose[1][0])+"  theta: "+str(pose[2][0])
		print(data_write)

					
	robot.move(0,0,0)


try: 
	while True:
		
		print("######### Enter your goal (x,y) :) ########## ")
		xd = float(input("enter x desired: "))
		yd = float(input("enter y desired: "))
		thetad = float(input("enter theta desired: "))	
		initOdometry()							
		g2g_oa(xd,yd,thetad)
				
## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()     
	print('\n\n		Stop!!! See you again!')
