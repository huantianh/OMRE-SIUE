import libomni as robot  #Library tha handles all the serial commands to arduino AtMega
import time
import serial
import math
import numpy as np

robot.enablePID(1)

count = 0

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
#odemetry position
current_x = 0
current_y = 0
current_theta = 0

#####################################################		Obstacle Position
def us0(d0,xc,yc,thetac):
	T_mat_robot_world= np.array([np.cos(thetac),-np.sin(thetac),0,xc,np.sin(thetac),np.cos(thetac),0,yc,0,0,1,0,0,0,0,1]).reshape(4,4)
	T_mat_sen0_robot = np.array([-1,0,0,-0.33,0,-1,0,0,0,0,1,0,0,0,0,1]).reshape(4,4)
	P_obs0_sen0		 = np.array([d0,0,0,1]).reshape(4,1)	
	P_obs0_robot 	 = np.dot(T_mat_sen0_robot,P_obs0_sen0)
	P_obs0_world	 = np.dot(T_mat_robot_world,P_obs0_robot)	
	P_obs0			 = np.concatenate([P_obs0_robot,P_obs0_world])
	return 	P_obs0_robot
	
def us1(d1,xc,yc,thetac):
	T_mat_robot_world= np.array([np.cos(thetac),-np.sin(thetac),0,xc,np.sin(thetac),np.cos(thetac),0,yc,0,0,1,0,0,0,0,1]).reshape(4,4)
	T_mat_sen1_robot = np.array([0,-1,0,0,1,0,0,0.33,0,0,1,0,0,0,0,1]).reshape(4,4)
	P_obs1_sen1		 = np.array([d1,0,0,1]).reshape(4,1)	
	P_obs1_robot 	 = np.dot(T_mat_sen1_robot,P_obs1_sen1)
	P_obs1_world	 = np.dot(T_mat_robot_world,P_obs1_robot)	
	P_obs1			 = np.concatenate([P_obs1_robot,P_obs1_world])
	return 	P_obs1_robot 
	
def us2(d2,xc,yc,thetac):
	T_mat_robot_world= np.array([np.cos(thetac),-np.sin(thetac),0,xc,np.sin(thetac),np.cos(thetac),0,yc,0,0,1,0,0,0,0,1]).reshape(4,4)
	T_mat_sen2_robot = np.array([np.sqrt(3)/2,-1/2,0,0.3556*(np.sqrt(3)/2),1/2,np.sqrt(3)/2,0,0.3556/2,0,0,1,0,0,0,0,1]).reshape(4,4)
	P_obs2_sen2		 = np.array([d2,0,0,1]).reshape(4,1)	
	P_obs2_robot 	 = np.dot(T_mat_sen2_robot,P_obs2_sen2)
	P_obs2_world	 = np.dot(T_mat_robot_world,P_obs2_robot)
	P_obs2			 = np.concatenate([P_obs2_robot,P_obs2_world])
	return 	P_obs2_robot 

def us3(d3,xc,yc,thetac):
	T_mat_robot_world= np.array([np.cos(thetac),-np.sin(thetac),0,xc,np.sin(thetac),np.cos(thetac),0,yc,0,0,1,0,0,0,0,1]).reshape(4,4)
	T_mat_sen3_robot = np.array([1,0,0,0.33,0,1,0,0,0,0,1,0,0,0,0,1]).reshape(4,4)
	P_obs3_sen3		 = np.array([d3,0,0,1]).reshape(4,1)	
	P_obs3_robot 	 = np.dot(T_mat_sen3_robot,P_obs3_sen3)
	P_obs3_world	 = np.dot(T_mat_robot_world,P_obs3_robot)	
	P_obs3			 = np.concatenate([P_obs3_robot,P_obs3_world])
	return 	P_obs3_robot 

def us4(d4,xc,yc,thetac):
	T_mat_robot_world= np.array([np.cos(thetac),-np.sin(thetac),0,xc,np.sin(thetac),np.cos(thetac),0,yc,0,0,1,0,0,0,0,1]).reshape(4,4)
	T_mat_sen4_robot = np.array([np.sqrt(3)/2,1/2,0,0.3556*(np.sqrt(3)/2),-1/2,np.sqrt(3)/2,0,-0.3556/2,0,0,1,0,0,0,0,1]).reshape(4,4)
	P_obs4_sen4		 = np.array([d4,0,0,1]).reshape(4,1)	
	P_obs4_robot 	 = np.dot(T_mat_sen4_robot,P_obs4_sen4)
	P_obs4_world	 = np.dot(T_mat_robot_world,P_obs4_robot)	
	P_obs4			 = np.concatenate([P_obs4_robot,P_obs4_world])
	return 	P_obs4_robot 

def us5(d5,xc,yc,thetac):
	T_mat_robot_world=  np.array([np.cos(thetac),-np.sin(thetac),0,xc,np.sin(thetac),np.cos(thetac),0,yc,0,0,1,0,0,0,0,1]).reshape(4,4)
	T_mat_sen5_robot = np.array([0,1,0,0,-1,0,0,-0.33,0,0,1,0,0,0,0,1]).reshape(4,4)
	P_obs5_sen5		 = np.array([d5,0,0,1]).reshape(4,1)	
	P_obs5_robot 	 = np.dot(T_mat_sen5_robot,P_obs5_sen5)
	P_obs5_world	 = np.dot(T_mat_robot_world,P_obs5_robot)
	P_obs5			 = np.concatenate([P_obs5_robot,P_obs5_world])
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

try: 

	while True:
		#~ print("current x     = "+str(current_x))
		#~ print("current y     = "+str(current_y))
		#~ print("current theta = "+str(current_theta))
		xc = current_x
		yc = current_y
		thetac = current_theta
		
		pose = odemetryCalc(xc,yc,thetac)
		
		for x in range(6):
			us[x] = robot.ultrasonic(x)
			print(str(us[0])+", "+str(us[1])+", "+str(us[2])+", "+str(us[3])+", "+str(us[4])+", "+str(us[5]))
			
#########################################		sensor 0
		#~ if (us[0] > 0) and (us[0]<= 0.2):
			#~ d0 = us[0]
			#~ obs0pos = us0(d0,xc,yc,thetac)		
			
			#~ xd0 = -obs0pos.item(0)
			#~ yd0 = -obs0pos.item(1) 	
			#~ robot.move(xd0,yd0,0)
			
##########################################		sensor 1 (left)
		if(us[1] > 0) and (us[1]<= 0.2):
			d1 = us[1]
			obs1pos = us1(d1,xc,yc,thetac)		
			#~ print(obs1pos)
			
			xd1 = -obs1pos.item(0)
			yd1 = -obs1pos.item(1) 
			robot.move(xd1,yd1,0)

##########################################		sensor 2		
		elif(us[2] != 0):
			d2 = us[2]
			obs2pos = us2(d2,xc,yc,thetac)		
			xd2 = -obs2pos.item(0)
			yd2 = -obs2pos.item(1)  				
			robot.move(xd2,yd2,0)
			
			if (us[1] != 0):
				robot.move(0,0,-1.57)
				time.sleep(.14)
			elif (us[1] == 0) and (us[5] == 0):
				robot.move(0,0,-1.57)
				time.sleep(.14)
			elif (us[1] != 0) and (us[5] != 0):
				if (us[1] > us[5]):
					robot.move(0,0,1.57)
				else:
					robot.move(0,0,-1.57)
				
			if (us[5] != 0):
				robot.move(0,0,1.57)
				time.sleep(.14)
			elif (us[1] == 0) and (us[5] == 0):
				robot.move(0,0,-1.57)
				time.sleep(.14)
			elif (us[1] != 0) and (us[5] != 0):
				if (us[1] > us[5]):
					robot.move(0,0,1.57)
				else:
					robot.move(0,0,-1.57)
##########################################		sensor 3 (front)
		elif(us[3] != 0):
			d3 = us[3]
			obs3pos = us3(d3,xc,yc,thetac)		
			xd3 = -obs3pos.item(0)
			yd3 = -obs3pos.item(1)  				
			robot.move(xd3,yd3,0)
			
			if (us[1] != 0):
				robot.move(0,0,-1.57)
				time.sleep(.14)
			elif (us[1] == 0) and (us[5] == 0):
				robot.move(0,0,-1.57)
				time.sleep(.14)
			elif (us[1] != 0) and (us[5] != 0):
				if (us[1] > us[5]):
					robot.move(0,0,1.57)
				else:
					robot.move(0,0,-1.57)
				
			if (us[5] != 0):
				robot.move(0,0,1.57)
				time.sleep(.14)
			elif (us[1] == 0) and (us[5] == 0):
				robot.move(0,0,-1.57)
				time.sleep(.14)
			elif (us[1] != 0) and (us[5] != 0):
				if (us[1] > us[5]):
					robot.move(0,0,1.57)
				else:
					robot.move(0,0,-1.57)
				
##########################################		sensor 4
		elif(us[4] != 0):
			d4 = us[4]
			obs4pos = us4(d4,xc,yc,thetac)		
			xd4 = -obs4pos.item(0)
			yd4 = -obs4pos.item(1)
			robot.move(xd4,yd4,0)
			
			if (us[1] != 0):
				robot.move(0,0,-1.57)
				time.sleep(.14)
			elif (us[1] == 0) and (us[5] == 0):
				robot.move(0,0,-1.57)
				time.sleep(.14)
			elif (us[1] != 0) and (us[5] != 0):
				if (us[1] > us[5]):
					robot.move(0,0,1.57)
				else:
					robot.move(0,0,-1.57)
				
			if (us[5] != 0):
				robot.move(0,0,1.57)
				time.sleep(.14)
			elif (us[1] == 0) and (us[5] == 0):
				robot.move(0,0,-1.57)
				time.sleep(.14)
			elif (us[1] != 0) and (us[5] != 0):
				if (us[1] > us[5]):
					robot.move(0,0,1.57)
				else:
					robot.move(0,0,-1.57)
				
##########################################		sensor 5 (right)		
		elif(us[5] > 0) and (us[5]<= 0.2):
			d5 = us[5]
			obs5pos = us5(d5,xc,yc,thetac)		
			
			xd5 = -obs5pos.item(0)
			yd5 = -obs5pos.item(1)
			robot.move(xd5,yd5,0)

##########################################		No Obsatcle			
		else:
			robot.motorVelocity(0,50,-50)
			
		current_x = pose.item(0)
		current_y = pose.item(1)
		current_theta = pose.item(2)	
			
## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()     
	print('		Stop!!! See you again!')
	
	
		#~ d0 = us[0]
			#~ d1 = us[1]
			#~ d2 = us[2]
			#~ d3 = us[3]
			#~ d4 = us[4]
			#~ d5 = us[5]
	
			#~ obs0pos = us0(d0,xc,yc,thetac)
			#~ obs1pos = us1(d1,xc,yc,thetac)
			#~ obs2pos = us2(d2,xc,yc,thetac)
			#~ obs3pos = us3(d3,xc,yc,thetac)
			#~ obs4pos = us4(d4,xc,yc,thetac)
			#~ obs5pos = us5(d5,xc,yc,thetac)
			
			#~ print(str(obs0pos[0])+"  ,  "+str(obs1pos[0])+"  ,  "+str(obs2pos[0])+"  ,  "+str(obs3pos[0])+"  ,  "+str(obs4pos[0])+"  ,  "+str(obs5pos[0]))
	
			#~ run_vector = np.sum([obs1pos,obs2pos,obs3pos,obs4pos,obs5pos],axis=0)
			#~ print(str(run_vector[0])+" ,  "+str(run_vector[1])) 
			
			#~ xd = run_vector[0]
			#~ yd = run_vector[1]
			
			#~ if (us[3] > 0) and (us[3]<= 0.2):
				#~ xd3 = obs3pos.item(0)
				#~ yd3 = obs3pos.item(1) 
				#~ robot.move(-xd3,-yd3,0)
				#~ time.sleep(.14)
				
				#~ if (us[1] != 0):
					#~ robot.move(0,0,-1.57)
					#~ time.sleep(.14)
				#~ elif (us[5] != 0):
					#~ robot.move(0,0,1.57)
					#~ time.sleep(0.6)
				#~ elif (us[1] != 0) and (us[5] != 0):
					#~ robot.move(0,0,3.14)
					#~ time.sleep(0.6)
				#~ elif (us[1] == 0):
					#~ robot.move(0,0,1.57)
					#~ time.sleep(.14)
			#~ elif ((us[2] > 0) and (us[2]<= 0.2)) or ((us[4] > 0) and (us[4]<= 0.2)):
				#~ robot.move(-xd,-yd,0)
			#~ elif ((us[1] > 0) and (us[1]<= 0.2)) or ((us[5] > 0) and (us[5]<= 0.2)):
				#~ robot.move(0,-yd,0)
			#~ else:
				#~ robot.motorVelocity(0,50,-50)	
