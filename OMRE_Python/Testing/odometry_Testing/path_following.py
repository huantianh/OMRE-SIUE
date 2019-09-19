import libomni as robot  #Library tha handles all the serial commands to arduino AtMega
import numpy as np
import time,os,serial,math

#folder where saving all the data
save_folder1 = "testing_g2g_data/close_loop/"
save_folder2 = "testing_g2g_data/open_loop/"
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


def path_f(xd,yd,thetad):
	global current_x
	global current_y
	global current_theta

	file = open(save_folder2 + "Circle_Open" +".txt","a")
	
	while True:

		xc = current_x
		yc = current_y
		thetac = current_theta
		
		vr = 0.01
		
		if (0.5-xd == 0):
			x_dot_d = vr
			y_dot_d = 0
			theta_dot_d = 0.01
		
		if (yd-0.5 == 0):
			x_dot_d = 0
			y_dot_d = vr	
			theta_dot_d = 0.01
			
		else:
			dydx = (0.5-xd) / (yd-0.5)
			x_dot_d = vr / np.sqrt(1+dydx*dydx)
			y_dot_d = dydx*x_dot_d
			theta_dot_d = 0.01
		
		q_dot_d = np.array([x_dot_d,y_dot_d,theta_dot_d]).reshape(3,1)
		
		j = (2*np.pi*0.03/60)*np.array([(2/3)*np.sin(thetad+np.pi/3),(-2/3)*np.sin(thetad),(2/3)*np.sin(thetad-np.pi/3),(-2/3)*np.cos(thetad+np.pi/3),(2/3)*np.cos(thetad),(-2/3)*np.cos(thetad-np.pi/3),-1/(3*0.19),-1/(3*0.19),-1/(3*0.19)]).reshape(3,3)
		j_inv = np.linalg.inv(j).reshape(3,3)
		
		K = 5
				
		e = np.array([(xc - xd),(yc-yd),(thetac-thetad)]).reshape(3,1) 			
		
		s = -K*e + q_dot_d
				
		vm = np.dot( j_inv, s, out=None)
		
		motor_spd_vec = vm
		
		wheel1RPM = motor_spd_vec[0] # motor 2 speed [rpm]
		wheel0RPM = motor_spd_vec[1] # motor 1 speed [rpm]
		wheel2RPM = motor_spd_vec[2] # motor 3 speed [rpm]
		

		robot.motorVelocity(int(wheel0RPM),int(wheel1RPM),int(wheel2RPM))
		
		pose = odometryCalc(xc,yc,thetac)	
		
		current_x = pose.item(0)
		current_y = pose.item(1)
		current_theta = pose.item(2)
		
		delta = np.sqrt(((xd-current_x)**2)+((yd-current_y)**2))
		# ~ print(delta)
		
		data_write = "x: "+str(pose[0][0])+"  y: "+str(pose[1][0])+"  theta: "+str(pose[2][0])
		print(data_write)
		file.writelines(str(pose[0][0])+" , "+str(pose[1][0])+" , "+str(pose[2][0])+"\n")
		
		if delta < 0.05:	
			file.close()
			robot.stop()	
			break

try: 
	while True:
		mode = str(input("Enter mode: s for start "))
			
		if mode == 's':	
				
			for x in np.arange(0,1,0.008):
	
				b = 0.5
				a = 0.5
				R = 0.5
				
				w = R*R - (x-a)*(x-a)
				
				y = b + np.sqrt(w)
		
				xd = x	
				yd = y
				thetad = 0
											
				initOdometry()							
				path_f(float(xd),float(yd),float(thetad))
				
			for x1 in np.arange(1,0,-0.008):
	
				b = 0.5
				a = 0.5
				R = 0.5
				
				w = R*R - (x1-a)*(x1-a)
				
				y1 = b - np.sqrt(w)					
				
				xd1 = x1	
				yd1 = y1
				thetad = 0
				
				# ~ print(xd)								
				initOdometry()							
				path_f(float(xd1),float(yd1),float(thetad))
	

## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()    
	#~ file.close() 
	print('\n\n		Stop!!! See you again!')
