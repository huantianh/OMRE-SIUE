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
omega = 0

output_vol1 = 0
output_vol2 = 0
output_vol3 = 0
last_output_vol1 = 0
last_output_vol2 = 0
last_output_vol3 = 0



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
			delay = 0.001
			speed = 0.5
			###### filter gain
			dt_tau = 0.7           #1   
			tau = 2                #0.05
			############################################################		gains
			Kv = 10
			Kp = 10
			
			m = 4.49
			Jn = 0.8289
			Kt = 0.3535
			
			test_t = 8
			
			while t < test_t:
				
				start = time.time()
						
				xd = R*np.sin(speed*t)
				yd = R*np.cos(speed*t)
				thetad = 0
				
				file = open(save_folder + "Dynamics_Based"+"_Kv_"+str(Kv)+"_Kp_"+str(Kp)+"_delay_"+str(delay)+"_speed_"+str(speed)+"_test_t_"+str(test_t)+".txt","a")
				
				xc = current_x
				yc = current_y
				thetac = current_theta
				
				
				q = np.array([xc,yc,thetac]).reshape(3,1)
				
				q_dot = np.array([vel_x,vel_y,omega]).reshape(3,1)
				
				print(q_dot)
				
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
				
				########################################################		Parameters
				r = 0.03
				l = 0.19
							
				########################################################		Dynamics Based Controller
					
				C = np.array([0, -m*omega, 0, m*omega, 0, 0, 0, 0, 0]).reshape(3,3)
				M = np.array([m, 0, 0, 0, m, 0, 0, 0, Jn]).reshape(3,3)
				
				e1 = qd_dot - q_dot
				e2 = q - qd
				
				J1 = qd_ddot + np.dot(Kv,e1,out = None) + np.dot(Kp,e2,out = None)
				
				f = np.dot(C,q_dot,out = None) + np.dot(M,J1,out = None)
				
				Jf = np.array([0, np.sqrt(3)/2, -np.sqrt(3)/2, -1, 1/2, 1/2, l, l, l]).reshape(3,3)
				Jf_inv = np.linalg.inv(Jf).reshape(3,3)
				
				Rot = np.array([np.cos(thetac), np.sin(thetac), 0, -np.sin(thetac), np.cos(thetac), 0, 0, 0, 1]).reshape(3,3)
				J2 = np.dot(Rot,f, out = None)
				
				torque = r * np.dot(Jf_inv,J2,out = None)
			
				
				current_ref = torque * (1/Kt)
				current_ref_convert = current_ref * 10
				
				# ~ print(current_ref_convert)
				
				current_input1 = int(current_ref_convert[0]) 
				current_input2 = int(current_ref_convert[1]) 
				current_input3 = int(current_ref_convert[2])
				
				robot.motor_current_pid(current_input1,current_input2,current_input3) 
				
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
				# ~ data_pwm = str(current_input1)+" , "+str(current_input2)+" , "+str(current_input3)
				data_pwm = str(1)+" , "+str(2)+" , "+str(3)
				
				########################################################		
				m1_vol = float(robot.motor_voltage(0))
				m2_vol = float(robot.motor_voltage(1))
				m3_vol = float(robot.motor_voltage(2))
				
				input_vol1 = float(m1_vol)
				input_vol2 = float(m2_vol)
				input_vol3 = float(m3_vol)
				
				output_vol1 += alpha*(input_vol1 - last_output_vol1)
				output_vol2 += alpha*(input_vol2 - last_output_vol2)
				output_vol3 += alpha*(input_vol3 - last_output_vol3)
				
				m1_vol_f = output_vol1
				m2_vol_f = output_vol2
				m3_vol_f = output_vol3
				
				data_vol = str(m1_vol_f)+' , ' +str(m2_vol_f)+ ' , ' +str(m3_vol_f)
				
				
				########################################################
				m1_cur = robot.motor_current(0)
				m2_cur = robot.motor_current(1)
				m3_cur = robot.motor_current(2)
				data_cur = str(m1_cur)+' , '+str(m2_cur)+' , '+str(m3_cur)
				
				# ~ print(data_cur)
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
				omega = theta_dot
				data_vel = str(vel_x)+' , '+str(vel_y)+' , '+str(theta_dot) 
				data_acc = str(acc_x)+' , '+str(acc_y)+' , '+str(acc_z)
				
				########################################################		Recording data
				time_running = time.time()
				# ~ data_pose = str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])
				data_pos = str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])
				# ~ data_pos  = str(pos_x)+" , "+str(pos_y)+" , "+str(pose[2][0])
				data_qd   = str(xd)+" , "+str(yd)+" , "+str(thetad) 
				
				file.writelines(str(data_pos)+" , "+str(data_vel)+' , '+str(data_acc)+" , "+str(data_pwm)+" , "+str(data_rpm)+" , "+str(data_vol)+" , "+str(data_cur)+" , "+str(time_running)+"\n")	
				
				########################################################		Remembering value for new loop			
				time.sleep(delay)
				elapsed_time = (time.time() - start)
				t = t + elapsed_time
				
				# ~ print(elapsed_time)
				
				last_output1 = output1
				last_output2 = output2
				last_output3 = output3
				
				last_output_vol1 = output_vol1
				last_output_vol2 = output_vol2
				last_output_vol3 = output_vol3
				
				last_yaw     = current_theta 
				
			robot.stop()
				

## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()    
	#~ file.close() 
	print('\n\n		Stop!!! See you again!')
