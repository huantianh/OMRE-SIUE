import libomni as robot  #Library tha handles all the serial commands to arduino AtMega
import pyrealsense2 as rs
import numpy as np
import time,os,serial,math

#folder where saving all the data
save_folder = "traf_data/"
current_directory = os.getcwd()
save_directory = os.path.join(current_directory, save_folder)

#odometry setups
oldEncoder0 = 0;oldEncoder1 = 0;oldEncoder2 = 0;newEncoder0 = 0;newEncoder1 = 0;newEncoder2 = 0;

#RealSense
vel_x = 0;vel_y = 0;vel_z = 0;pos_x = 0;pos_y = 0;pos_z = 0;acc_x = 0;acc_y = 0;acc_z = 0;

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


global output1
global output2
global output3
global dt
global last_output1
global last_output2
global last_output3
global last_yaw
global omega
omega = 0
yaw = 0
theta_dot = 0
output1 = 0
output2 = 0
output3 = 0
last_output1 = 0
last_output2 = 0
last_output3 = 0
last_yaw = 0
dt      = 0




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
	# ~ w_theta = float(theta2[7]) 
	# ~ x_theta = float(theta2[1]) 
	# ~ y_theta = float(theta2[3]) 
	# ~ z_theta = float(theta2[5]) 
	w_theta =  data.rotation.w
	x_theta = -data.rotation.z
	y_theta =  data.rotation.x
	z_theta = -data.rotation.y
		
	theta_rs = np.arccos(w_theta) * 2 * 180/np.pi
	# ~ print(theta_rs)
	pitch =  -np.arcsin(2.0 * (x_theta*z_theta - w_theta*y_theta));
	roll  =  np.arctan2(2.0 * (w_theta*x_theta + y_theta*z_theta), w_theta*w_theta - x_theta*x_theta - y_theta*y_theta + z_theta*z_theta);
	yaw   =  -np.arctan2(2.0 * (w_theta*z_theta + x_theta*y_theta), w_theta*w_theta + x_theta*x_theta - y_theta*y_theta - z_theta*z_theta);
	
			

try: 
	while True:
		mode = str(input("Enter s to start "))

		if mode == 's':


			############################################################		Reset encoder and Enable RealSense
			initOdometry()
			odometry_RealSense()

			R = 0.8
			############################################################		Value for t and delta_t
			t = 0
			delay = 0.01
			speed = 1
			###### filter gain
			dt_tau = 0.5           #1   
			tau = 2                #0.05
			############################################################		gains
			K1 = 100
			K2 = 5
			K3 = 20
			
			test_t = 20
			
			while t < test_t:
				
				start = time.time()
						
				xd = R*np.sin(speed*t)
				yd = R*np.cos(speed*t)-0.8
				thetad = 0
				
				file = open(save_folder + "Robust"+"_K1_"+str(K1)+"_K2_"+str(K2)+"_K3_"+str(K3)+"_delay_"+str(delay)+"_speed_"+str(speed)+"_test_t_"+str(test_t)+".txt","a")
				
				xc = current_x
				yc = current_y
				thetac = current_theta
				
				q = np.array([xc,yc,thetac]).reshape(3,1)
				q_dot = np.array([vel_x,vel_y,omega]).reshape(3,1)
				qd = np.array([xd,yd,thetad]).reshape(3,1)
										
				########################################################		qd_dot
				xd_dot =  R*speed*np.cos(speed*t)
				yd_dot = -R*speed*np.sin(speed*t)
				thetad_dot = 0
				
				qd_dot = np.array([xd_dot,yd_dot,thetad_dot]).reshape(3,1)
				
				########################################################		qd_ddot
				xd_ddot = -R*speed*speed*np.sin(speed*t)
				yd_ddot = -R*speed*speed*np.cos(speed*t)
				thetad_ddot = 0
				
				qd_ddot = np.array([xd_ddot,yd_ddot,thetad_ddot]).reshape(3,1)				
				
				########################################################		qd_dddot
				xd_dddot = -R*speed*speed*speed*np.cos(speed*t)
				yd_dddot = R*speed*speed*speed*np.sin(speed*t)
				thetad_dddot = 0
				
				qd_dddot = np.array([xd_dddot,yd_dddot,thetad_dddot]).reshape(3,1)
				
				########################################################		J_inverse
				r = 0.03
				l = 0.19
				j = (2*np.pi*r/60)*np.array([(2/3)*np.sin(thetac+np.pi/3),(-2/3)*np.sin(thetac),(2/3)*np.sin(thetac-np.pi/3),(-2/3)*np.cos(thetac+np.pi/3),(2/3)*np.cos(thetac),(-2/3)*np.cos(thetac-np.pi/3),-1/(3*l),-1/(3*l),-1/(3*l)]).reshape(3,3)
				j_inv = np.linalg.inv(j).reshape(3,3)
				j_trans = np.transpose(j).reshape(3,3)
				
				# ~ j_dot = np.array([(r*np.pi*np.cos(thetac+np.pi/3))/45, -(r*np.pi*np.cos(thetac))/45, (r*np.pi*np.cos(thetac-np.pi/3))/45, (r*np.pi*np.sin(thetac+np.pi/3))/45, -(r*np.pi*np.sin(thetac))/45, (r*np.pi*np.sin(thetac-np.pi/3))/45, 0, 0, 0]).reshape(3,3)
				# ~ j_inv_dot = np.array([(30*np.cos(thetac+np.pi/3))/(r*np.pi), (30*np.sin(thetac+np.pi/3))/(r*np.pi), 0, -(30*np.cos(thetac))/(r*np.pi), -(30*np.sin(thetac))/(r*np.pi), 0, (30*np.cos(thetac-np.pi/3))/(r*np.pi), -(30*np.cos(thetac+np.pi/6))/(r*np.pi), 0]).reshape(3,3)
				
				j_dot = theta_dot*np.array([(r*np.pi*np.cos(thetac+np.pi/3))/45, -(r*np.pi*np.cos(thetac))/45, (r*np.pi*np.cos(thetac-np.pi/3))/45, (r*np.pi*np.sin(thetac+np.pi/3))/45, -(r*np.pi*np.sin(thetac))/45, (r*np.pi*np.sin(thetac-np.pi/3))/45, 0, 0, 0]).reshape(3,3)
				j_inv_dot = theta_dot*np.array([(30*np.cos(thetac+np.pi/3))/(r*np.pi), (30*np.sin(thetac+np.pi/3))/(r*np.pi), 0, -(30*np.cos(thetac))/(r*np.pi), -(30*np.sin(thetac))/(r*np.pi), 0, (30*np.cos(thetac-np.pi/3))/(r*np.pi), -(30*np.cos(thetac+np.pi/6))/(r*np.pi), 0]).reshape(3,3)
				
				
				########################################################		Robust Controller
				############			Parameters
				a1 = 12.17; b1 = 224.46; a2 = 4.74; b2 = 10.08; c2 = 0.32;
				
				A1 = np.array([a1,0,0,0,a1,0,0,0,a1]).reshape(3,3)
				B1 = np.array([b1,0,0,0,b1,0,0,0,b1]).reshape(3,3)
				B1_inv = np.linalg.inv(B1).reshape(3,3)
				
				e1 = np.array([(xc-xd),(yc-yd),(thetac-thetad)]).reshape(3,1) 
				
				########################################################		reading actual RPM
				############		actual RPM (Wc)
				m1_rpm = float(robot.rpm(0))
				m2_rpm = float(robot.rpm(1))
				m3_rpm = float(robot.rpm(2))
				
				alpha = dt_tau / tau
				
				input1 = float(m1_rpm)
				input2 = float(m2_rpm)
				input3 = float(m3_rpm)
				
				output1 += alpha*(input1 - last_output1)
				output2 += alpha*(input2 - last_output2)
				output3 += alpha*(input3 - last_output3)
				
				m1_rpm_f = output1
				m2_rpm_f = output2
				m3_rpm_f = output3
				
				data_rpm = str(m1_rpm_f)+' , ' +str(m2_rpm_f)+ ' , ' +str(m3_rpm_f)
				# ~ data_rpm = str(m1_rpm)+' , ' +str(m2_rpm)+ ' , ' +str(m3_rpm)
				
				########################################################		wd
				wd = np.dot(j_inv, -K1*e1 + qd_dot,out = None)
				
				############		commanded (Wd)
				c_rpm2 = float(wd[0])
				c_rpm1 = float(wd[1])
				c_rpm3 = float(wd[2])
				
				e2 = np.array([(m2_rpm_f-c_rpm2),(m1_rpm_f-c_rpm1),(m3_rpm_f-c_rpm3)]).reshape(3,1) 
				
				u11 = np.dot(j_inv,K1*j,out=None)
				u12 = A1 - u11 - K2	
				u1 = np.dot(u12,e2,out=None)
				
				u2 = K3*np.sign(e2)
				
				u31 = np.dot(A1,j_inv*K1,out=None)
				u32 = np.dot(j_inv_dot,K1,out=None)
				u33 = np.dot(j_inv,K1**2,out=None)
				u34 = u31 + u32 - u33 + j_trans				
				u3 = np.dot(u34,e1,out=None)
				
				u411 = np.dot(j_inv,qd_dot,out=None)
				u41 = np.dot(A1,u411,out=None)
				u42 = np.dot(j_inv_dot,qd_dot,out=None)
				u43 = np.dot(j_inv,qd_ddot,out=None)
				u4 = u41 + u42 + u43
				
				u = np.dot(B1_inv,(u1 -u2 -u3 +u4), out=None)  

				########################################################		commanded RPM
				wheel1RPM = float(u[0]) # motor 2 speed [rpm]
				wheel0RPM = float(u[1]) # motor 1 speed [rpm]
				wheel2RPM = float(u[2]) # motor 3 speed [rpm]
				
				data_c_rpm = str(wheel0RPM)+' , ' +str(wheel1RPM)+ ' , ' +str(wheel2RPM)
				# ~ print(data_c_rpm)
				########################################################		Sending input RPM to Arduino
				robot.motor_rpm(int(wheel0RPM),int(wheel1RPM),int(wheel2RPM))
				
				########################################################		odometry using encoder
				pose = odometryCalc(xc,yc,thetac)	
				pos  = odometry_RealSense()
				
				current_x = pose.item(0)
				current_y = pose.item(1)
				current_theta = pose.item(2)
				
				########################################################		odometry using RealSense
				# ~ current_x = pos_x
				# ~ current_y = pos_y
				# ~ current_theta = pose.item(2)		
				
				########################################################		RealSense velocities
				dt = (time.time() - start)
				# ~ dyaw = yaw - last_yaw
				dyaw = current_theta - last_yaw
				theta_dot = dyaw/dt
				theta_dot = omega
				print(theta_dot)
				data_vel = str(vel_x)+' , '+str(vel_y)+' , '+str(theta_dot) 
				
				########################################################		Recording data
				time_running = time.time()
				data_pose = str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])
				data_pos  = str(pos_x)+" , "+str(pos_y)
				data_qd   = str(xd)+" , "+str(yd)+" , "+str(thetad) 
				file.writelines(str(data_pose)+" , "+str(data_pos)+" , "+str(data_rpm)+" , "+str(data_c_rpm)+" , "+str(data_vel)+" , "+str(data_qd)+" , "+str(time_running)+"\n")	
				########################################################		Remembering value for new loop			
				time.sleep(delay)
				elapsed_time = (time.time() - start)
				t = t + elapsed_time
				
				last_output1 = output1
				last_output2 = output2
				last_output3 = output3
				# ~ last_yaw     = yaw  
				last_yaw     = current_theta 
				# ~ print(elapsed_time)
			robot.stop()
				

## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()    
	#~ file.close() 
	print('\n\n		Stop!!! See you again!')
