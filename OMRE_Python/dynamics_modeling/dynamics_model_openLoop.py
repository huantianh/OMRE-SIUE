import libomni as robot  #Library tha handles all the serial commands to arduino AtMega
import pyrealsense2 as rs
import numpy as np
import time,os,serial
import math as m

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
global last_yaw
global vx
global vy
global omega

last_yaw = 0;vx = 0; vy =0;omega = 0;
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
	global yaw
	
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
			
			t = 0
			test_t = 5
			delay = 0.01
		
			
			while t < test_t:
				
				start = time.time()

				x_dot = 0;              #max 0.7618
				y_dot = 0.65;                 #max 0.6597
				theta_dot = 0;             #max 3.4723


				############		J, J_inverse, J_dot, J_transpose
				r = 0.03
				l = 0.19

				J2_inv = np.array([1/r,0,0, 0,1/r,0, 0,0,1/r]).reshape(3,3)
				J1     = np.array([np.sqrt(3)/2,-1/2,-l, 0,1,-l, -np.sqrt(3)/2,-1/2,-l]).reshape(3,3)

				v_robot= np.array([x_dot, y_dot, theta_dot]).reshape(3,1)  
				J2_inv_J1 = np.dot( J2_inv, J1, out=None).reshape(3,3) 
				phi_dot =  np.dot( J2_inv_J1, v_robot, out=None).reshape(3,1) 

				rpm1 = phi_dot.item(0)*60/(2*np.pi)
				rpm0 = phi_dot.item(1)*60/(2*np.pi)
				rpm2 = phi_dot.item(2)*60/(2*np.pi)
				
				wheel0RPM = rpm0
				wheel1RPM = rpm1
				wheel2RPM = rpm2
				
				# ~ if t < 2:
					# ~ wheel0RPM = 0
					# ~ wheel1RPM = 0
					# ~ wheel2RPM = 0
				# ~ if t > 2 and t < 7:	
					# ~ wheel0RPM = rpm0
					# ~ wheel1RPM = rpm1
					# ~ wheel2RPM = rpm2
				# ~ if t > 7:
					# ~ wheel0RPM = 0
					# ~ wheel1RPM = 0
					# ~ wheel2RPM = 0		
				
				
				li = 46.85                  #reduction of motors
				Kt1 = 1.16516 / 5.6			#Motor Torque Constant
				Kt2 = 1.16516 / 5.6			#Motor Torque Constant
				Kt3 = 1.16516 / 5.6			#Motor Torque Constant
				r = 0.03					#Wheel Radius
				l = 0.19                    #distance from wheel to CG
				
				# ~ file = open(save_folder + "MotorsModelTest_X_"+str(x_dot)+"_test_t_"+str(test_t)+".txt","a")
				file = open(save_folder + "MotorsModelTest_Y_"+str(y_dot)+"_test_t_"+str(test_t)+".txt","a")
				# ~ file = open(save_folder + "MotorsModelTest_Theta_"+str(theta_dot)+"_test_t_"+str(test_t)+".txt","a")
				
				xc = current_x
				yc = current_y
				thetac = current_theta
				
				robot.motor_pwm(int(wheel0RPM),int(wheel1RPM),int(wheel2RPM))
				
				########################################################		Caculating
				##########				Wheel RPM
				m1_rpm = robot.rpm(0)
				m2_rpm = robot.rpm(1)
				m3_rpm = robot.rpm(2)
				data_rpm = str(m1_rpm)+' , '+str(m2_rpm)+' , '+str(m3_rpm)
				##########				Motor Current	
				m1_cur = robot.motor_current(0)
				m2_cur = robot.motor_current(1)
				m3_cur = robot.motor_current(2)
				data_cur = str(m1_cur)+' , '+str(m2_cur)+' , '+str(m3_cur)
				# ~ print(data_cur)
				##########				Motor Voltage	
				# ~ m1_vol = robot.motor_voltage()
				# ~ m2_vol = robot.motor_voltage(1)
				# ~ m3_vol = robot.motor_voltage(2)
				# ~ print(m1_vol)
				# ~ data_vol = str(m1_vol)+' , '+str(m2_vol)+' , '+str(m3_vol)
				# ~ print(data_cur)
				##########				Rotation Torque
				T1 = li*Kt1*m1_cur
				T2 = li*Kt2*m2_cur
				T3 = li*Kt3*m3_cur
				##########				Wheel Traction Force
				f1 = T1/r
				f2 = T2/r
				f3 = T3/r
				##########				Robot Traction Force
				Fv = f2*np.cos(30) - f3*np.cos(30)
				Fvn = f1 - f2*np.sin(30) - f3*np.sin(30)
				R_f = (-f1-f2-f3)*l
				data_trac = str(Fv)+' , '+str(Fvn)+' , '+str(R_f)
				
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
				dt = (time.time() - start)
				dyaw = yaw - last_yaw
				vel_ang = dyaw/dt
				data_vel = str(vel_x)+' , '+str(vel_y)+' , '+str(vel_ang)
				# ~ print(data_vel)
				
				########################################################		Recording data
				time_running = time.time()
				data_pose = str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])
				data_pos  = str(pos_x)+" , "+str(pos_y)
				
				# ~ print(data_pose)
				file.writelines(str(data_pose)+" , "+str(data_pos)+" , "+str(data_rpm)+" , "+str(data_vel)+" , "+str(data_cur)+" , "+str(time_running)+"\n")
					
				########################################################		Remembering value for new loop			
				time.sleep(delay)
				elapsed_time = (time.time() - start)
				last_yaw = yaw
				t = t + elapsed_time
				
			robot.stop()
				

## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()    
	#~ file.close() 
	print('\n\n		Stop!!! See you again!')
