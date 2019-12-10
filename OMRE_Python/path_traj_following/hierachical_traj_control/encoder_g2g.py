import libomni as robot  #Library tha handles all the serial commands to arduino AtMega
import pyrealsense2 as rs
import numpy as np
import time,os,serial,math

#folder where saving all the data
save_folder= "traf_data/"
current_directory = os.getcwd()
save_directory = os.path.join(current_directory, save_folder)

#odometry setups
oldEncoder0 = 0
oldEncoder1 = 0
oldEncoder2 = 0
newEncoder0 = 0
newEncoder1 = 0
newEncoder2 = 0

vel_x = 0
vel_y = 0
vel_z = 0
pos_x = 0
pos_y = 0
pos_z = 0
acc_x = 0
acc_y = 0
acc_z = 0


####################################################		Reset encoder
def initOdometry():
	global oldEncoder0 
	global oldEncoder1 
	global oldEncoder2 
	oldEncoder0 = robot.encoder(0)
	oldEncoder1 = robot.encoder(1)
	oldEncoder2 = robot.encoder(2)
	
#odometry position
current_x = 0
current_y = 0
current_theta = 0 

######################################################		Odometry	
def odometryCalc(xk,yk,thetak,l=0.19, N=2249, r=0.03):
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


def g2g(xd,yd,thetad):
	global current_x
	global current_y
	global current_theta
	
	# ~ file = open(save_folder2 + "x_"+str(xd)+",y_"+str(yd)+",theta_"+str(thetad)+".txt","a+")
	# ~ file = open(save_folder2 + "Triangle_Open" +".txt","a")
	file = open(save_folder + "Square_Open" +".txt","a")
	
	while True:

		xc = current_x
		yc = current_y
		thetac = current_theta


		inv_rotation_mat= np.array([np.cos(thetac), np.sin(thetac), 0, -np.sin(thetac), np.cos(thetac), 0, 0, 0, 1]).reshape(3,3)
		d = np.sqrt(((xd-xc)**2)+((yd-yc)**2))

		phi = math.atan2((yd-yc),(xd-xc))

		vel_global = np.array([ d*np.cos(phi), d*np.sin(phi), -1*(current_theta-thetad)])[:,None]
			
		vel_local = np.dot(inv_rotation_mat, vel_global)
		
		# ~ print(vel_local)
				
		v_x = vel_local[0]
		v_y = vel_local[1]
		v_theta = vel_local[2]
		
		
		robot.move(v_x, v_y, v_theta)
		
		pose = odometryCalc(xc,yc,thetac)
		
		current_x = pose.item(0)
		current_y = pose.item(1)
		current_theta = pose.item(2)
		
		delta = np.sqrt(((xd-current_x)**2)+((yd-current_y)**2)) #< 0.1	
		
		data_write = "x: "+str(pose[0][0])+"  y: "+str(pose[1][0])+"  theta: "+str(pose[2][0])
		print(data_write)
		file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+"\n")
		
	
		if delta < 0.01:	
			file.close()
			robot.stop()	
			break


try: 
	while True:
		
		print("######### Enter your goal (x,y) :) ########## ")
		xd = float(input("enter x desired: "))
		yd = float(input("enter y desired: "))
		thetad = float(input("enter theta desired: "))	
		initOdometry()							
		g2g(xd,yd,thetad)	

			
## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()    
	#~ file.close() 
	print('\n\n		Stop!!! See you again!')
