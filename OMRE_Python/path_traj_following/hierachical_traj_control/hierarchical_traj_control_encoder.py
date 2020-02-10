import libomni as robot  #Library tha handles all the serial commands to arduino AtMega
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

#pid controller
integral = np.array([0,0,0])[:,None]
preError = np.array([0,0,0])[:,None]

global last_wd
global last_vd
global dt
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

try: 
	while True:
		mode = str(input("Enter s to start "))
		
		if mode == 's':

			initOdometry()
			t = 0
			del_t  = 0.1
			
			while True:
				start = time.time()
				
				########################################################			Gain K
				k  = 30
				
				Kx = k
				Ky = k
				Kz = k
				
				file = open(save_folder + "Circle_TraF"+"_K_"+str(k)+".txt","a")
				
				########################################################			Path
				xd = np.sin(0.1*t)
				yd = np.cos(0.1*t)
				thetad = 0
				
				########################################################			Parameters
				a1 = 12.17; b1 = 224.46; a2 = 4.74; b2 = 10.08; c2 = 0.32;
			
				########################################################			Initial Pose
				xc = current_x
				yc = current_y
				thetac = current_theta
				
				########################################################			Q_dot and J 
				############		q_dot_d
				x_dot_d =  0.1*np.cos(0.1*t)
				y_dot_d = -0.1*np.sin(0.1*t)
				theta_dot_d = 0.01
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
				wd_dot = (last_wd - wd)/del_t
				# ~ print(str(wd) +" , "+str(wd_dot))
				# ~ print(wd_dot)
				
				############		commanded RPM (Wd)
				c_rpm2 = float(wd[0])
				c_rpm1 = float(wd[1])
				c_rpm3 = float(wd[2])
				# ~ print(str(c_rpm1)+" , "+str(c_rpm2)+" , "+str(c_rpm3)) 
				
				############		actual RPM (Wc)
				m1_rpm = int(robot.rpm(0))
				m2_rpm = int(robot.rpm(1))
				m3_rpm = int(robot.rpm(2))
				data_rpm = str(m1_rpm)+' , ' +str(m2_rpm)+ ' , ' +str(m3_rpm)
				# ~ print(data_rpm)
				
				########################################################			Calculate V
				e2 = np.array([(m1_rpm-c_rpm1),(m2_rpm-c_rpm2),(m3_rpm-c_rpm3)]).reshape(3,1) 
				z1 = np.dot(j,e2,out=None).reshape(3,1) 
				j1 = np.dot(j_inv,j_dot,out=None) 
				j2 = np.dot(j_inv,e1,out=None) 
				
				vd = 1/b1*a1*wd + 1/b1*wd_dot - 1/b1*np.dot(j1,e2,out=None) - 1/b1*j2
				vd_dot = (last_vd-vd)/del_t
				# ~ print(str(vd)+" , "+str(vd_dot)) 

				########################################################			Input U
				j3 = np.dot(j_trans,z1,out=None)
				u1 = 1/b2*(vd_dot + a2*vd + b1*j3)
				c3 = 1/c2
				u  = np.sign(u1) * (np.abs(u1)) ** (c3)
				# ~ print(str(u) +' , '+ str(u1))
				print(str(int(u[1]))+" , "+str(int(u[0]))+" , "+str(int(u[2])))
				
				m2_pwm_control = float(u[0]) # motor 2 speed [rpm]
				m1_pwm_control = float(u[1]) # motor 1 speed [rpm]
				m3_pwm_control = float(u[2]) # motor 3 speed [rpm]
				
				robot.motor_pwm(int(m1_pwm_control),int(m2_pwm_control),int(m3_pwm_control))
				
				########################################################			odometry using encoder
				pose = odometryCalc(xc,yc,thetac)	
				
				current_x = pose.item(0)
				current_y = pose.item(1)
				current_theta = pose.item(2)
				
				########################################################			odometry using RealSense
				# ~ current_x = pos_x
				# ~ current_y = pos_y
				# ~ current_theta = pose.item(2)

				########################################################			RealSense velocities
				vel = 0
				# ~ vel = np.sqrt(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z)
				# ~ vel = str(vel_x)+" , "+str(vel_y)
				# ~ print(vel)
			
				########################################################			Recording data
				time_running = time.time()		
				data_pose = "x: "+str(pose[0][0])+"  y: "+str(pose[1][0])+"  theta: "+str(pose[2][0])
				# ~ print(data_pose)
				# ~ file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+" , "+str(pos_x)+" , "+str(pos_y)+" , "+str(m1_rpm)+" , "+str(m2_rpm)+" , "+str(m3_rpm)+" , "+str(time_running)+ "\n")
				file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+" , "+str(pos_x)+" , "+str(pos_y)+" , "+str(m1_rpm)+" , "+str(m2_rpm)+" , "+str(m3_rpm)+" , "+str(float(u[1]))+" , "+str(float(u[0]))+" , "+str(float(u[2]))+" , "+str(vel_x)+" , "+str(vel_y)+" , "+str(vel)+" , "+str(time_running)+ "\n")
				
				########################################################			Preparing for new loop			
				dt = (time.time() - start)
				# ~ print(dt)
				last_wd = wd
				last_vd = vd
				time.sleep(del_t)
				t = t + 0.1
				

## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()    
	#~ file.close() 
	print('\n\n		Stop!!! See you again!')