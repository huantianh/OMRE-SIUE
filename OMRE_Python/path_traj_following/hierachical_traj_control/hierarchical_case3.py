import libomni as robot  #Library that handles all the serial commands to arduino AtMega
import pyrealsense2 as rs
import numpy as np
import time,os,serial,math

#folder where saving all the data
save_folder = "traf_data/"
current_directory = os.getcwd()
save_directory = os.path.join(current_directory, save_folder)

#odometry setups
oldEncoder0 = 0;oldEncoder1 = 0;oldEncoder2 = 0;newEncoder0 = 0;newEncoder1 = 0;newEncoder2 = 0

#RealSense
vel_x = 0;vel_y = 0;vel_z = 0;pos_x = 0;pos_y = 0;pos_z = 0;acc_x = 0;acc_y = 0;acc_z = 0

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()
# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
# Start streaming with requested config
pipe.start(cfg)

#pid controller
integral = np.array([0,0,0])[:,None]
preError = np.array([0,0,0])[:,None]

global output1
global output2
global output3
global last_wd
global last_vd
global dt
global last_output1
global last_output2
global last_output3
output1 = 0
output2 = 0
output3 = 0
last_output1 = 0
last_output2 = 0
last_output3 = 0
last_vd = 0
last_wd = 0
dt      = 0


#####################################################		Reset encoder
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

			initOdometry()
			odometry_RealSense()
			
			R = 0.5
			speed = 1
			###### time gain
			elapsed_time = 0.1
			t = 0
			delay = 0.1
			test_t = 60
			
			###### derivative gain
			del_t  = delay
			# ~ del_t  = 0.08
			
			###### filter gain
			dt_tau = delay         #1   
			tau = 2                #0.05
			
			while t < test_t:
				start = time.time()
				
				########################################################			Gain K
				k  = 0.5
				Kx = k
				Ky = k
				Kz = k
				
				file = open(save_folder + "Case3"+"_K_"+str(k)+"_delay_"+str(delay)+"_speed_"+str(speed)+".txt","a")
				
				########################################################			Path
				xd = R*np.sin(speed*t)
				yd = R*np.cos(speed*t)
				thetad = 0
				
				########################################################			Parameters
				a1 = 12.17; b1 = 224.46; a2 = 4.74; b2 = 10.08; c2 = 0.32;
			
				########################################################			Initial Pose
				xc = current_x
				yc = current_y
				thetac = current_theta
				
				########################################################			Q_dot and J 
				############		q_dot_d
				x_dot_d =  R*speed*np.cos(speed*t)
				y_dot_d = -R*speed*np.sin(speed*t)
				theta_dot_d = 0.1
				q_dot_d = np.array([x_dot_d,y_dot_d,theta_dot_d]).reshape(3,1)
				
				############		J, J_inverse, J_dot, J_transpose
				r = 0.03
				l = 0.19
				j = (2*np.pi*r/60)*np.array([(2/3)*np.sin(thetac+np.pi/3),(-2/3)*np.sin(thetac),(2/3)*np.sin(thetac-np.pi/3),(-2/3)*np.cos(thetac+np.pi/3),(2/3)*np.cos(thetac),(-2/3)*np.cos(thetac-np.pi/3),-1/(3*l),-1/(3*l),-1/(3*l)]).reshape(3,3)
				j_inv = np.linalg.inv(j).reshape(3,3)
				j_dot = (2*np.pi*r/60)*np.array([(2/3)*np.cos(thetac+np.pi/3)*theta_dot_d,(-2/3)*np.cos(thetac)*theta_dot_d,(2/3)*np.cos(thetac-np.pi/3)*theta_dot_d,(2/3)*np.sin(thetac+np.pi/3)*theta_dot_d,(-2/3)*np.sin(thetac)*theta_dot_d,(2/3)*np.sin(thetac-np.pi/3)*theta_dot_d,0,0,0]).reshape(3,3)
				j_trans = np.transpose(j).reshape(3,3)
				# ~ print(str(j) +" , "+ str(j_dot))
				
				########################################################			Calculate W				
				K = np.array([Kx,0,0,0,Ky,0,0,0,Kz]).reshape(3,3)
				e1 = np.array([(xc-xd),(yc-yd),(thetac-thetad)]).reshape(3,1) 						
				w1 = np.dot(-K,e1,out=None) + q_dot_d
				
				############		Wd vector		
				wd = np.dot( j_inv, w1, out=None).reshape(3,1) 		
				# ~ wd_dot = (last_wd - wd)/del_t
				wd_dot = (last_wd - wd)/elapsed_time
				# ~ print(str(wd) +" , "+str(wd_dot))
				# ~ print(wd_dot)
				
				############		commanded (Wd)
				c_rpm2 = float(wd[0])
				c_rpm1 = float(wd[1])
				c_rpm3 = float(wd[2])
				# ~ print(str(c_rpm1)+" , "+str(c_rpm2)+" , "+str(c_rpm3)) 
				
				############		actual RPM (Wc)
				m1_rpm = int(robot.rpm(0))
				m2_rpm = int(robot.rpm(1))
				m3_rpm = int(robot.rpm(2))
				
				# ~ alpha = del_t / tau
				alpha = elapsed_time / tau
				
				input1 = m1_rpm
				input2 = m2_rpm
				input3 = m3_rpm
				
				output1 = output1 + alpha*(input1 - output1)
				output2 = output2 + alpha*(input2 - output2)
				output3 = output3 + alpha*(input3 - output3)
				
				# ~ output1 += alpha*(input1 - last_output1)
				# ~ output2 += alpha*(input2 - last_output2)
				# ~ output3 += alpha*(input3 - last_output3)
				
				m1_rpm_f = output1
				m2_rpm_f = output2
				m3_rpm_f = output3
				
				data_rpm = str(m1_rpm_f)+' , ' +str(m2_rpm_f)+ ' , ' +str(m3_rpm_f)
				# ~ print(data_rpm)
				
				########################################################			Calculate V
				# ~ e2 = np.array([(m1_rpm_f-c_rpm1),(m2_rpm_f-c_rpm2),(m3_rpm_f-c_rpm3)]).reshape(3,1) 
				e2 = np.array([(m1_rpm-c_rpm1),(m2_rpm-c_rpm2),(m3_rpm-c_rpm3)]).reshape(3,1) 
				z1 = np.dot(j,e2,out=None).reshape(3,1) 
				j1 = np.dot(j_inv,j_dot,out=None) 
				j2 = np.dot(j_inv,e1,out=None) 
				
				vd = 1/b1*a1*wd + 1/b1*wd_dot - 1/b1*np.dot(j1,e2,out=None) - 1/b1*j2
				# ~ vd_dot = (last_vd-vd)/del_t
				vd_dot = (last_vd-vd)/elapsed_time
				# ~ print(str(vd)+" , "+str(vd_dot)) 

				########################################################			Input U
				j3 = np.dot(j_trans,z1,out=None)
				u1 = 1/b2*(vd_dot + a2*vd + b1*j3)
				c3 = 1/c2
				u  = np.sign(u1) * (np.abs(u1)) ** (c3)
				# ~ print(str(u) +' , '+ str(u1))
				# ~ print(str(int(u[1]))+" , "+str(int(u[0]))+" , "+str(int(u[2])))
				
				m2_pwm_control = float(u[0]) # motor 2 speed [rpm]
				m1_pwm_control = float(u[1]) # motor 1 speed [rpm]
				m3_pwm_control = float(u[2]) # motor 3 speed [rpm]
				
				########################################################			Sending input U to Arduino
				robot.motor_pwm(int(m1_pwm_control),int(m2_pwm_control),int(m3_pwm_control))
				
				########################################################		    Getting RPM from Arduino 
				m1_rpm_u = int(robot.rpm(0))											
				m2_rpm_u = int(robot.rpm(1))
				m3_rpm_u = int(robot.rpm(2))
				data_rpm_u = str(m1_rpm_u)+' , ' +str(m2_rpm_u)+ ' , ' +str(m3_rpm_u)
				# ~ print(data_rpm_u)
				
				########################################################			odometry using encoder
				pose = odometryCalc(xc,yc,thetac)	
				pos  = odometry_RealSense()
				
				current_x = pose.item(0)
				current_y = pose.item(1)
				current_theta = pose.item(2)
				
				########################################################			odometry using RealSense
				# ~ current_x = pos_x
				# ~ current_y = pos_y
				# ~ current_theta = pose.item(2)

				########################################################			RealSense velocities
				vel = np.sqrt(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z)
				# ~ vel = str(vel_x)+" , "+str(vel_y)
				# ~ print(vel)
			
				########################################################			Recording data
				time_running = time.time()		
				data_pose = "x: "+str(pose[0][0])+"  y: "+str(pose[1][0])+"  theta: "+str(pose[2][0])
				# ~ print(data_pose)
				# ~ file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+" , "+str(pos_x)+" , "+str(pos_y)+" , "+str(m1_rpm)+" , "+str(m2_rpm)+" , "+str(m3_rpm)+" , "+str(time_running)+ "\n")
				file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+" , "+str(pos_x)+" , "+str(pos_y)+" , "+str(m1_rpm_u)+" , "+str(m2_rpm_u)+" , "+str(m3_rpm_u)+" , "+str(float(u[1]))+" , "+str(float(u[0]))+" , "+str(float(u[2]))+" , "+str(vel_x)+" , "+str(vel_y)+" , "+str(vel)+" , "+str(time_running)+ "\n")
				
				########################################################		Preparing for new loop			
				time.sleep(delay)
				elapsed_time = (time.time() - start)
				t = t + elapsed_time
				
				last_wd = wd
				last_vd = vd
				last_output1 = output1
				last_output2 = output2
				last_output3 = output3
				
			robot.stop()
				

## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()    
	#~ file.close() 
	print('\n\n		Stop!!! See you again!')
