import libomni as robot  #Library tha handles all the serial commands to arduino AtMega
import pyrealsense2 as rs
import numpy as np
import time,os,serial,math

#folder where saving all the data
save_folder = "save_data/"
current_directory = os.getcwd()
save_directory = os.path.join(current_directory, save_folder)

#odometry setups
oldEncoder0 = 0;oldEncoder1 = 0;oldEncoder2 = 0;newEncoder0 = 0;newEncoder1 = 0;newEncoder2 = 0;

#RealSense
vel_x = 0;vel_y = 0;vel_z = 0;pos_x = 0;pos_y = 0;pos_z = 0;acc_x = 0;acc_y = 0;acc_z = 0;

m1_rpm = 0;m2_rpm = 0;m3_rpm = 0;

global m1_cur
global m2_cur
global m3_cur

m1_cur = 0;m2_cur = 0;m3_cur = 0;

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()
# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
# Start streaming with requested config
pipe.start(cfg)

#pid controller
global integral
global preError
integral = np.array([0,0,0])[:,None]
preError = np.array([0,0,0])[:,None]

########################################################################		Reset encoder
def initOdometry():
	global oldEncoder0 
	global oldEncoder1 
	global oldEncoder2 
	oldEncoder0 = robot.encoder(0)
	oldEncoder1 = robot.encoder(1)
	oldEncoder2 = robot.encoder(2)
	
#odometry position
global current_x
global current_y
global current_theta
current_x = 0
current_y = 0
current_theta = 0 

########################################################################		Encoder Odometry	
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

########################################################################		RealSense Odometry
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
	global theta_rs
	
	frames = pipe.wait_for_frames()
	pose = frames.get_pose_frame()
	data = pose.get_pose_data()
	
	velocity = data.velocity
	position = data.translation
	acceleration = data.acceleration
	rotation = data.rotation
	
	####################################################################		get Velocity data
	vel1 = str(velocity)	
	vel2 = vel1.replace(', ',' ').split(' ')
	# ~ print(vel2[1] + " , " + vel2[3] + " , " + vel2[5])
	vel_x = -float(vel2[5]) 
	vel_y = -float(vel2[1]) 
	vel_z = float(vel2[3]) 
	
	####################################################################		get Postion data
	pos1 = str(position)	
	pos2 = pos1.replace(', ',' ').split(' ')
	# ~ print(pos2[1] + " , " + pos2[3] + " , " + pos2[5])
	pos_x = -float(pos2[5]) 
	pos_y = -float(pos2[1]) 
	pos_z = float(pos2[3]) 
	
	####################################################################		get Acceleration data
	acc1 = str(acceleration)	
	acc2 = acc1.replace(', ',' ').split(' ')
	# ~ print(acc2[1] + " , " + acc2[3] + " , " + acc2[5])
	acc_x = -float(acc2[5]) 
	acc_y = -float(acc2[1]) 
	acc_z = float(acc2[3])
	
	####################################################################		get Rotation data
	theta1 = str(rotation)	
	theta2 = theta1.replace(', ',' ').split(' ')
	# ~ print(acc2[1] + " , " + acc2[3] + " , " + acc2[5])
	w_theta = float(theta2[7]) 
	x_theta = float(theta2[1]) 
	y_theta = float(theta2[3]) 
	z_theta = float(theta2[5]) 
		
	# ~ sin_rs = 2 * (w_theta * z_theta + x_theta * y_theta)
	# ~ cos_rs = 1 - 2 * (y_theta * y_theta + z_theta * z_theta)	
	# ~ theta_rs = -np.arctan2(sin_rs,cos_rs) 
	theta_rs = np.arccos(w_theta) * 2
	# ~ print(theta_rs)
			

try: 
	while True:
		mode = str(input("Enter s to start "))

		if mode == 's':


			############################################################		Reset encoder and Enable RealSense
			initOdometry()
			odometry_RealSense()
			
			t = 0
			test_t = 5
			delay = 0.01
			# ~ vx = 0.31416
			# ~ vx = 0.502656
			vx = 0.659736
			
			while t < test_t:
				
				start = time.time()
				
				file = open(save_folder + "Dynamics"+"_Vx_"+str(vx)+"_test_t_"+str(test_t)+".txt","a")
				
				xc = current_x
				yc = current_y
				thetac = current_theta
				
				########################################################		Sending input RPM to Arduino
			
				# ~ wheel0RPM = 0
				# ~ wheel1RPM = 100
				# ~ wheel2RPM = -100
				
				# ~ wheel0RPM = 0
				# ~ wheel1RPM = 160
				# ~ wheel2RPM = -160
				
				wheel0RPM = 0
				wheel1RPM = 210
				wheel2RPM = -210
				
				robot.motor_rpm(int(wheel0RPM),int(wheel1RPM),int(wheel2RPM))
				
				########################################################		reading actual RPM
				m1_rpm = robot.rpm(0)
				m2_rpm = robot.rpm(1)
				m3_rpm = robot.rpm(2)
				data_rpm = str(m1_rpm)+' , ' +str(m2_rpm)+ ' , ' +str(m3_rpm)
					
				m1_cur = robot.motor_current(0)/100
				m2_cur = robot.motor_current(1)/100
				m3_cur = robot.motor_current(2)/100
				data_cur = str(m1_cur)+' , ' +str(m2_cur)+ ' , ' +str(m3_cur)
				print(data_cur)
				
				########################################################		odometry using encoder
				pose = odometryCalc(xc,yc,thetac)	
				pos  = odometry_RealSense()
				
				# ~ current_x = pose.item(0)
				# ~ current_y = pose.item(1)
				# ~ current_theta = pose.item(2)
				
				########################################################		odometry using RealSense
				current_x = pos_x
				current_y = pos_y
				current_theta = pose.item(2)		
				
				########################################################		RealSense velocities
				vel = np.sqrt(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z)
				
				########################################################		Recording data
				time_running = time.time()
				data_pose = "x: "+str(pose[0][0])+"  y: "+str(pose[1][0])+"  theta: "+str(pose[2][0])
				# ~ print(data_pose)
				file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+" , "+str(pos_x)+" , "+str(pos_y)+" , "+str(m1_rpm)+" , "+str(m2_rpm)+" , "+str(m3_rpm)+" , "+str(vel_x)+" , "+str(vel_y)+" , "+str(vel)+" , "+str(m1_cur)+" , "+str(m2_cur)+" , "+str(m3_cur)+" , "+str(time_running)+"\n")
					
				########################################################		Remembering value for new loop			
				time.sleep(delay)
				elapsed_time = (time.time() - start)
				t = t + elapsed_time
				
			robot.stop()
				

## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()    
	#~ file.close() 
	print('\n\n		Stop!!! See you again!')
