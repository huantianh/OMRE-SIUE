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
global integral
global preError
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
			
			###### radius and speed
			R = 0.8
			speed = 0.4
			
			###### time gain
			t = 0
			delay = 0.01           #0.01
			test_t = 30
			
			###### filter gain
			dt_tau = 0.5           #1   
			tau = 2                #0.05
			
			while t < test_t:
				start = time.time()
				
				#######################################################			Gain K
				kp  = 5
				ki  = 0
				kd  = 0
				
				file = open(save_folder + "Inf3"+"_Kp_"+str(kp)+"_Ki_"+str(ki)+"_Kd_"+str(kd)+"_delay_"+str(delay)+"_speed_"+str(speed)+".txt","a")
				
				########################################################			Path
				xd = np.cos(speed*t)-1
				yd = np.sin(speed*2*t)/2
				thetad = 0
				
				########################################################			Parameters
				a1 = 12.17; b1 = 224.46; a2 = 4.74; b2 = 10.08; c2 = 0.32;
			
				########################################################			Initial Pose
				xc = current_x
				yc = current_y
				thetac = current_theta
				
				########################################################			Q_dot and J 
				############		qd_dot
				xd_dot = -speed*np.sin(speed*t)
				yd_dot = speed*np.cos(speed*2*t)
				thetad_dot = 0
				qd_dot = np.array([xd_dot,yd_dot,thetad_dot]).reshape(3,1)
				
				############		J, J_inverse, J_dot, J_transpose
				r = 0.03
				l = 0.19
				j = (2*np.pi*r/60)*np.array([(2/3)*np.sin(thetac+np.pi/3),(-2/3)*np.sin(thetac),(2/3)*np.sin(thetac-np.pi/3),(-2/3)*np.cos(thetac+np.pi/3),(2/3)*np.cos(thetac),(-2/3)*np.cos(thetac-np.pi/3),-1/(3*l),-1/(3*l),-1/(3*l)]).reshape(3,3)
				j_inv = np.linalg.inv(j).reshape(3,3)
				j_dot = (2*np.pi*r/60)*np.array([(2/3)*np.cos(thetac+np.pi/3)*thetad_dot,(-2/3)*np.cos(thetac)*thetad_dot,(2/3)*np.cos(thetac-np.pi/3)*thetad_dot,(2/3)*np.sin(thetac+np.pi/3)*thetad_dot,(-2/3)*np.sin(thetac)*thetad_dot,(2/3)*np.sin(thetac-np.pi/3)*thetad_dot,0,0,0]).reshape(3,3)
				j_trans = np.transpose(j).reshape(3,3)
				# ~ print(str(j) +" , "+ str(j_dot))
				
				########################################################			Calculate W				
				Kp = kp
				Ki = ki
				Kd = kd
				
				setPoint     = np.array([xd,yd,thetad])[:,None]
				currentPoint = np.array([xc,yc,thetac])[:,None]
				e1           = currentPoint - setPoint
				preError     = e1
				integral     = integral + e1
				derivative   = e1 - preError
			
				output = Kp*e1 + Ki*integral + Kd*derivative	
				e1_dot = (-output).reshape(3,1)
				s = e1_dot + qd_dot
				
				############		Wd vector			
				wd = np.dot( j_inv, s, out=None).reshape(3,1) 		
				wd_dot1 = np.sqrt(3)*R*speed*(333.333333333333*(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac) - np.cos(thetac + np.pi/6)) - 333.333333333333*(speed*np.cos(speed*t) + np.sin(speed*t))*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + (np.sin(thetac + np.pi/6) + np.cos(thetac))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) - 7.91700075e+19*(speed*np.cos(speed*t) + np.sin(speed*t))*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2)/np.pi
				wd_dot2 = np.sqrt(3)*R*speed*(333.333333333333*(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6)) + 1.3889475e+17*(speed*np.cos(speed*t) + np.sin(speed*t))*(2.39989872427384e-15*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3)) - 3.59984808641075e-15*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 + 333.333333333333*(speed*np.cos(speed*t) + np.sin(speed*t))*((np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6)) + (np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)))/np.pi
				wd_dot3 = 333.333333333333*np.sqrt(3)*R*speed*(-(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + (speed*np.cos(speed*t) + np.sin(speed*t))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/np.pi
				wd_dot = np.array([wd_dot1,wd_dot2,wd_dot3]).reshape(3,1)

				
				############		commanded (Wd)
				c_rpm2 = float(wd[0])
				c_rpm1 = float(wd[1])
				c_rpm3 = float(wd[2])
			
				
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
				
				########################################################			Calculate V
				e2 = np.array([(m2_rpm_f-c_rpm2),(m1_rpm_f-c_rpm1),(m3_rpm_f-c_rpm3)]).reshape(3,1) 
				# ~ e2 = np.array([(m2_rpm-c_rpm2),(m1_rpm-c_rpm1),(m3_rpm-c_rpm3)]).reshape(3,1) 

				z1 = np.dot(j,e2,out=None).reshape(3,1) 
				j1 = np.dot(j_inv,j_dot,out=None) 
				j2 = np.dot(j_inv,e1,out=None) 
				
				vd = 1/b1*a1*wd + 1/b1*wd_dot - 1/b1*np.dot(j1,e2,out=None) - 1/b1*j2
				vd_dot1 = np.sqrt(3)*R*speed*(-0.00445513677269892*speed*(666.666666666667*(-speed*np.sin(speed*t) + np.cos(speed*t))*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + (np.sin(thetac + np.pi/6) + np.cos(thetac))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 + 666.666666666667*(-speed*np.sin(speed*t) + np.cos(speed*t))*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.cos(thetac) + np.cos(thetac + np.pi/3)) - (np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) + 1.58340015e+20*(-speed*np.sin(speed*t) + np.cos(speed*t))*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))**2/(np.cos(thetac) + np.cos(thetac + np.pi/3))**3 + 7.91700075e+19*(-speed*np.sin(speed*t) + np.cos(speed*t))*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) - 333.333333333333*(speed*np.cos(speed*t) + np.sin(speed*t))*(np.sin(thetac + np.pi/6) + np.cos(thetac))) + 18.0730048412486*(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac) - np.cos(thetac + np.pi/6)) - 18.0730048412486*(speed*np.cos(speed*t) + np.sin(speed*t))*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + (np.sin(thetac + np.pi/6) + np.cos(thetac))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) - 4.29251978648757e+18*(speed*np.cos(speed*t) + np.sin(speed*t))*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 - 1.48504559089964*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + (np.sin(thetac + np.pi/6) + np.cos(thetac))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))*np.sin(speed*t)/(np.cos(thetac) + np.cos(thetac + np.pi/3)) - 3.52713211708099e+17*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))*np.sin(speed*t)/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 - 1.48504559089964*(np.sin(thetac) - np.cos(thetac + np.pi/6))*np.cos(speed*t))/np.pi
				vd_dot2 = np.sqrt(3)*R*speed*(-0.00445513677269892*speed*(666.666666666667*(speed*np.sin(speed*t) - np.cos(speed*t))*(-(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3)) + (np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) + 2.777895e+17*(speed*np.sin(speed*t) - np.cos(speed*t))*(2.39989872427384e-15*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3)) - 3.59984808641075e-15*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))**2/(np.cos(thetac) + np.cos(thetac + np.pi/3))**3 + 1.3889475e+17*(speed*np.sin(speed*t) - np.cos(speed*t))*(2.39989872427384e-15*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3)) - 3.59984808641075e-15*np.sqrt(3))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) + 666.666666666667*(speed*np.sin(speed*t) - np.cos(speed*t))*((np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6)) + (np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 + 333.333333333333*(speed*np.cos(speed*t) + np.sin(speed*t))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3))) + 18.0730048412486*(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6)) + 7.53073646752205e+15*(speed*np.cos(speed*t) + np.sin(speed*t))*(2.39989872427384e-15*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3)) - 3.59984808641075e-15*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 + 18.0730048412486*(speed*np.cos(speed*t) + np.sin(speed*t))*((np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6)) + (np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) + 618795108259824.0*(2.39989872427384e-15*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3)) - 3.59984808641075e-15*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))*np.sin(speed*t)/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 + 1.48504559089964*((np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6)) + (np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))*np.sin(speed*t)/(np.cos(thetac) + np.cos(thetac + np.pi/3)) - 1.48504559089964*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6))*np.cos(speed*t))/np.pi
				vd_dot3 = np.sqrt(3)*R*speed*(1.48504559089964*speed*((speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) - (speed*np.cos(speed*t) + np.sin(speed*t))*(np.cos(thetac) + np.cos(thetac + np.pi/3))) - 18.0730048412486*(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + 18.0730048412486*(speed*np.cos(speed*t) + np.sin(speed*t))*(np.cos(thetac) + np.cos(thetac + np.pi/3)) + 1.48504559089964*(np.sin(thetac) + np.sin(thetac + np.pi/3))*np.cos(speed*t) + 1.48504559089964*(np.cos(thetac) + np.cos(thetac + np.pi/3))*np.sin(speed*t))/np.pi
				vd_dot = np.array([vd_dot1,vd_dot2,vd_dot3]).reshape(3,1)

				########################################################			Input U
				j3 = np.dot(j_trans,z1,out=None)
				u1 = 1/b2*(vd_dot + a2*vd + b1*j3)
				c3 = 1/c2
				u11 = np.sign(u1[0]) * (np.abs(u1[0])) ** (c3)
				u21 = np.sign(u1[1]) * (np.abs(u1[1])) ** (c3)
				u31 = np.sign(u1[2]) * (np.abs(u1[2])) ** (c3)
				
				u  = np.array([u11,u21,u31]).reshape(3,1)
				
				# ~ u  = np.sign(u1) * (np.abs(u1)) ** (c3)
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
				
				# ~ current_x = pose.item(0)
				# ~ current_y = pose.item(1)
				# ~ current_theta = pose.item(2)
				
				########################################################			odometry using RealSense
				current_x = pos_x
				current_y = pos_y
				current_theta = pose.item(2)

				########################################################			RealSense velocities
				vel = np.sqrt(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z)
				# ~ vel = str(vel_x)+" , "+str(vel_y)
				# ~ print(vel)
			
				########################################################			Recording data
				time_running = time.time()		
				data_pose = "x: "+str(pose[0][0])+"  y: "+str(pose[1][0])+"  theta: "+str(pose[2][0])
				# ~ print(data_pose)
				file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+" , "+str(pos_x)+" , "+str(pos_y)+" , "+str(m1_rpm_u)+" , "+str(m2_rpm_u)+" , "+str(m3_rpm_u)+" , "+str(float(u[1]))+" , "+str(float(u[0]))+" , "+str(float(u[2]))+" , "+str(vel_x)+" , "+str(vel_y)+" , "+str(vel)+" , "+str(xd)+" , "+str(yd)+" , "+str(thetad)+" , "+str(time_running)+"\n")
				# ~ file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+" , "+str(xd)+" , "+str(yd)+" , "+str(m1_rpm_u)+" , "+str(m2_rpm_u)+" , "+str(m3_rpm_u)+" , "+str(float(u[1]))+" , "+str(float(u[0]))+" , "+str(float(u[2]))+" , "+str(vel_x)+" , "+str(vel_y)+" , "+str(vel)+" , "+str(time_running)+ "\n")
				
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
