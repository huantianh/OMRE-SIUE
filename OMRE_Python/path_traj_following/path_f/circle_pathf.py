import libomni as robot  #Library tha handles all the serial commands to arduino AtMega
import pyrealsense2 as rs
import numpy as np
import time,os,serial,math

#folder where saving all the data
save_folder = "path_data/"
current_directory = os.getcwd()
save_directory = os.path.join(current_directory, save_folder)

#odometry setups
oldEncoder0 = 0
oldEncoder1 = 0
oldEncoder2 = 0
newEncoder0 = 0
newEncoder1 = 0
newEncoder2 = 0

#RealSense
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



#####################################################		Reset encoder
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
	
	#get Velocity data
	vel1 = str(velocity)	
	vel2 = vel1.replace(', ',' ').split(' ')
	# ~ print(vel2[1] + " , " + vel2[3] + " , " + vel2[5])
	vel_x = -float(vel2[5]) 
	vel_y = -float(vel2[1]) 
	vel_z = float(vel2[3]) 
	
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
	acc_x = -float(acc2[5]) 
	acc_y = -float(acc2[1]) 
	acc_z = float(acc2[3]) 

	#get Rotation data
	theta1 = str(rotation)	
	theta2 = theta1.replace(', ',' ').split(' ')
	# ~ print(acc2[1] + " , " + acc2[3] + " , " + acc2[5])
	w_theta = float(theta2[7]) 
	x_theta = float(theta2[1]) 
	y_theta = float(theta2[3]) 
	z_theta = float(theta2[5]) 
		
	sin_rs = 2 * (w_theta * z_theta + x_theta * y_theta)
	cos_rs = 1 - 2 * (y_theta * y_theta + z_theta * z_theta)
		
	# ~ theta_rs = -np.arctan2(sin_rs,cos_rs) 
	theta_rs = np.arccos(w_theta) * 2
	
######################################################################			Path_Following
def path_f(xd,yd,thetad,b,a,R,step):
	global current_x
	global current_y
	global current_theta
	
	delta_min = 0.1
	vr = 0.1
	k = 5
	Kx = k	
	Ky = k
	Kz = k

	file = open(save_folder + "Circle"+"_vr_"+str(vr)+"_K_"+str(k)+"_delta_"+str(delta_min)+'_R_'+str(R)+'_step_'+str(step)+".txt","a")
	# ~ file = open(save_folder2 + "Circle"+"_vr_"+str(vr)+"_K_"+str(Kx)+"_"+str(Ky)+"_"+str(Kz)+"_delta_"+str(delta_min)+".txt","a")
	
	while True:

		xc = current_x
		yc = current_y
		thetac = current_theta
		
		if (a-xd == 0):
			x_dot_d = vr
			y_dot_d = 0
			theta_dot_d = 0.1
			
		if (yd-b == 0):
			x_dot_d = 0
			y_dot_d = vr
			theta_dot_d = 0.1
						
		else:
			dydx = (a-xd)/(yd-b)
			x_dot_d = vr / np.sqrt(1+dydx*dydx)
			y_dot_d = dydx*x_dot_d
			theta_dot_d = 0.1
		
		q_dot_d = np.array([x_dot_d,y_dot_d,theta_dot_d]).reshape(3,1)
		
		j = (2*np.pi*0.03/60)*np.array([(2/3)*np.sin(thetad+np.pi/3),(-2/3)*np.sin(thetad),(2/3)*np.sin(thetad-np.pi/3),(-2/3)*np.cos(thetad+np.pi/3),(2/3)*np.cos(thetad),(-2/3)*np.cos(thetad-np.pi/3),-1/(3*0.19),-1/(3*0.19),-1/(3*0.19)]).reshape(3,3)
		j_inv = np.linalg.inv(j).reshape(3,3)
			
		K = np.array([Kx,0,0,0,Ky,0,0,0,Kz]).reshape(3,3)
				
		e = np.array([(xc - xd),(yc-yd),(thetac-thetad)]).reshape(3,1) 						
		
		s = np.dot(-K,e,out=None) + q_dot_d
		
		########################################################		Input W	(RPM)	
		w = np.dot( j_inv, s, out=None)	
		motor_spd_vec = w
		# ~ print(vm)
		
		########################################################		commanded RPM
		wheel1RPM = motor_spd_vec[0] # motor 2 speed [rpm]
		wheel0RPM = motor_spd_vec[1] # motor 1 speed [rpm]
		wheel2RPM = motor_spd_vec[2] # motor 3 speed [rpm]
		
		c_rpm1 = float(wheel0RPM)
		c_rpm2 = float(wheel1RPM)
		c_rpm3 = float(wheel2RPM)
			
		robot.motor_rpm(int(wheel0RPM),int(wheel1RPM),int(wheel2RPM))
		
		pose = odometryCalc(xc,yc,thetac)	
		pos  = odometry_RealSense()
		
		current_x = pose.item(0)
		current_y = pose.item(1)
		current_theta = pose.item(2)
		
		#use RealSense as feedback
		# ~ current_x = pos_x
		# ~ current_y = pos_y
		# ~ current_theta = theta_rs
		
		vel = np.sqrt(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z)
		# ~ vel = str(vel_x)+" , "+str(vel_y)
		# ~ print(vel)
		
		delta = np.sqrt(((xd-current_x)**2)+((yd-current_y)**2))
		# ~ print(delta)
		
		m1_rpm = robot.rpm(0)
		m2_rpm = robot.rpm(1)
		m3_rpm = robot.rpm(2)
		data_rpm = str(m1_rpm)+' , ' +str(m2_rpm)+ ' , ' +str(m3_rpm)
		# ~ print(data_rpm) 	
		
		# ~ start = time.time()
		time_running = time.time()
		# ~ print(time_running)
		
		data_pose = "x: "+str(pose[0][0])+"  y: "+str(pose[1][0])+"  theta: "+str(pose[2][0])
		print(data_pose)
		# ~ file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+" , "+str(pos_x)+" , "+str(pos_y)+" , "+str(m1_rpm)+" , "+str(m2_rpm)+" , "+str(m3_rpm)+" , "+str(time_running)+ "\n")
		# ~ file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+" , "+str(pos_x)+" , "+str(pos_y)+" , "+str(m1_rpm)+" , "+str(m2_rpm)+" , "+str(m3_rpm)+" , "+str(vel_x)+" , "+str(vel_y)+" , "+str(vel)+" , "+str(time_running)+ "\n")
		file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+" , "+str(pos_x)+" , "+str(pos_y)+" , "+str(m1_rpm)+" , "+str(m2_rpm)+" , "+str(m3_rpm)+" , "+str(c_rpm1)+" , "+str(c_rpm2)+" , "+str(c_rpm3)+" , "+str(vel_x)+" , "+str(vel_y)+" , "+str(vel)+" , "+str(time_running)+ "\n")
		
		if delta < delta_min:	
			file.close()
			robot.stop()	
			break

try: 
	while True:
		R = float(input("Enter R: "))		
		
		initOdometry()
		odometry_RealSense()
		
		b = 0
		a = 0
			
		step = 0.01
			
		# ~ y1 = np.arange(0,b*2,step)
		# ~ w1 = R*R - (y1-b)*(y1-b)
		# ~ x1 = a + np.sqrt(w1)
		# ~ y2 = np.arange(b*2,0,-step)
		# ~ y3 = np.append(y2,0)
		# ~ w2 = R*R - (y3-b)*(y3-b)
		# ~ x2 = a - np.sqrt(w2)

		delay = 0.1
		t = 0
		test_t = 30

		while t < test_t:
			start = time.time()
			
			y1 = np.arange(R,-R,-step)
			w1 = R*R - (y1-b)*(y1-b)
			x1 = a + np.sqrt(w1)

			y2 = np.arange(-R,R,step)
			y3 = np.append(y2,R)
			w2 = R*R - (y2-b)*(y2-b)
			x2 = a - np.sqrt(w2)

			x = np.concatenate((x1,x2))
			y = np.concatenate((y1,y3))
						
			for i,j in zip(x,y):
				xd = i
				yd = j
				thetad = 0
				path_f(float(xd),float(yd),float(thetad),float(b),float(a),float(R),float(step))
				
				# ~ time.sleep(delay)
			elapsed_time = (time.time() - start)
			t = t + elapsed_time
	
## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()    
	#~ file.close() 
	print('\n\n		Stop!!! See you again!')
