import libomni as robot  #Library tha handles all the serial commands to arduino AtMega
import pyrealsense2 as rs
import numpy as np
import time,os,serial,math

#folder where saving all the data
save_folder1 = "testing_g2g/close_loop/"
save_folder2 = "testing_g2g/open_loop/"
current_directory = os.getcwd()
save_directory1 = os.path.join(current_directory, save_folder1)
save_directory2 = os.path.join(current_directory, save_folder2)

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

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()
# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
# Start streaming with requested config
pipe.start(cfg)


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

def odometry_RealSense():
	global vel_x 
	global vel_y 
	global vel_z 
	global pos_x 
	global pos_y 
	global pos_z 
	global acc_x 
	global acc_y 
	global acc_z 
	
	frames = pipe.wait_for_frames()
	pose = frames.get_pose_frame()
	data = pose.get_pose_data()
	
	velocity = data.velocity
	position = data.translation
	acceleration = data.acceleration
	
	#get Velocity data
	vel1 = str(velocity)	
	vel2 = vel1.replace(', ',' ').split(' ')
	# ~ print(vel2[1] + " , " + vel2[3] + " , " + vel2[5])
	vel_x = float(vel2[1]) 
	vel_y = float(vel2[3]) 
	vel_z = float(vel2[5]) 
	
	#get Postion data
	pos1 = str(position)	
	pos2 = pos1.replace(', ',' ').split(' ')
	# ~ print(pos2[1] + " , " + pos2[3] + " , " + pos2[5])
	pos_x = -float(pos2[5]) 
	pos_y = -float(pos2[1]) 
	pos_z = float(pos2[3]) 
	
	#get Acceleration data
	acc1 = str(acceleration)	
	acc2 = acc1.replace(', ',' ').split(' ')
	# ~ print(acc2[1] + " , " + acc2[3] + " , " + acc2[5])
	acc_x = float(acc2[1]) 
	acc_y = float(acc2[3]) 
	acc_z = float(acc2[5]) 

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


def g2g_pid(xd,yd,thetad):

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
	min_distance = 0.01
	
	delta = np.sqrt(((xd-current_x)**2)+((yd-current_y)**2))
	

	while delta > min_distance:
		
		# ~ file = open(save_folder1 + "x_"+str(xd)+",y_"+str(yd)+",theta_"+str(thetad)+".txt","a+")
		# ~ file = open(save_folder1 + "Triangle_Closed" +".txt","a")
		file = open(save_folder1 + "Square_Closed" +".txt","a")
		
		xc = current_x
		yc = current_y
		thetac = current_theta
		
		pose = odometryCalc(xc,yc,thetac)
		
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
		
		v_x = vel_local[0]
		v_y = vel_local[1] 
		v_theta = vel_local[2] 
		
		robot.move(v_x,v_y,v_theta)	
		
		#Odometry
		current_x = pos_x
		current_y = pos_y
		current_theta = pose.item(2)

		delta = np.sqrt(((xd-current_x)**2)+((yd-current_y)**2))
				
		data_write = "x: "+str(pos_x)+"  y: "+str(pos_y)+"  theta: "+str(pose[2][0])
		print(data_write)
		file.writelines(str(pos_x)+" , "+str(pos_y)+" , "+str(pose[2][0])+"\n")
		file.close()
				
	robot.stop()

def g2g(xd,yd,thetad):
	global current_x
	global current_y
	global current_theta
	
	# ~ file = open(save_folder2 + "x_"+str(xd)+",y_"+str(yd)+",theta_"+str(thetad)+".txt","a+")
	# ~ file = open(save_folder2 + "Triangle_Open" +".txt","a")
	file = open(save_folder2 + "Square_Open" +".txt","a")
	
	while True:

		xc = current_x
		yc = current_y
		thetac = current_theta


		inv_rotation_mat= np.array([np.cos(thetac), np.sin(thetac), 0, -np.sin(thetac), np.cos(thetac), 0, 0, 0, 1]).reshape(3,3)
		d = np.sqrt(((xd-xc)**2)+((yd-yc)**2))

		phi = math.atan2((yd-yc),(xd-xc))

		vel_global = np.array([ d*np.cos(phi), d*np.sin(phi), -1*(current_theta-thetad)])[:,None]
			
		vel_local = np.dot(inv_rotation_mat, vel_global)
				
		v_x = vel_local[0]
		v_y = vel_local[1]
		v_theta = vel_local[2]
		
		robot.move(v_x, v_y, v_theta)
		
		pose = odometryCalc(xc,yc,thetac)
		pos  = odometry_RealSense()
		
		# ~ current_x = pose.item(0)
		# ~ current_y = pose.item(1)
		# ~ current_theta = pose.item(2)
	
		current_x = pos_x
		current_y = pos_y
		current_theta = pose.item(2)
	
		
		delta = np.sqrt(((xd-current_x)**2)+((yd-current_y)**2)) #< 0.1	
		
		data_write = "x: "+str(pose[0][0])+"  y: "+str(pose[1][0])+"  theta: "+str(pose[2][0])
		print(data_write)
		file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+" , "+str(pos_x)+" , "+str(pos_y)+"\n")
		
		if delta < 0.01:	
			file.close()
			robot.stop()	
			break


try: 
	while True:
		mode = str(input("Enter mode: g for regular g2g, p for PID "))
			
		if mode == 'g':	
			# ~ f = open("triangle_values.txt",'r')
			f = open("square_values.txt",'r')
			lines = f.readlines()
			xd = []
			yd = []
			thetad = []
			
			for line in lines:
				x = line.split(',')[0]
				y = line.split(',')[1]
				theta = line.split(',')[2]

				xd = x	
				yd = y
				thetad = theta
				initOdometry()
				odometry_RealSense()							
				g2g(float(xd),float(yd),float(thetad))
		
			# ~ print("######### Enter your goal (x,y) :) ########## ")
			# ~ xd = float(input("enter x desired: "))
			# ~ yd = float(input("enter y desired: "))
			# ~ thetad = float(input("enter theta desired: "))	
			# ~ initOdometry()							
			# ~ odometry_RealSense()
			# ~ g2g(xd,yd,thetad)			
		
		if mode == 'p':	
			# ~ f = open("triangle_values.txt",'r')
			f = open("square_values.txt",'r')
			lines = f.readlines()
			xd = []
			yd = []
			thetad = []
						
			for line in lines:
				x = line.split(',')[0]
				y = line.split(',')[1]
				theta = line.split(',')[2]
						
				xd = x	
				yd = y
				thetad = theta
				initOdometry()		
				odometry_RealSense()					
				g2g_pid(float(xd),float(yd),float(thetad))
							# ~ print("######### Enter your goal (x,y) :) ########## ")
			# ~ xd = float(input("enter x desired: "))
			# ~ yd = float(input("enter y desired: "))
			# ~ thetad = float(input("enter theta desired: "))	
			# ~ initOdometry()	
			# ~ odometry_RealSense()						
			# ~ g2g_pid(xd,yd,thetad)

## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()    
	#~ file.close() 
	print('\n\n		Stop!!! See you again!')
