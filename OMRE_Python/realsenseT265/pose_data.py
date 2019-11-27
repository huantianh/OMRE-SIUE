# First import the library
import pyrealsense2 as rs
import time
import libomni as robot
import numpy as np

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)

try:
	while True:
		
		frames = pipe.wait_for_frames()
		pose = frames.get_pose_frame()
		data = pose.get_pose_data()
		
		velocity = data.velocity
		position = data.translation
		acceleration = data.acceleration
		rotation = data.rotation
		
		# ~ print(rotation)
		
		#get Velocity data
		vel1 = str(velocity)	
		vel2 = vel1.replace(', ',' ').split(' ')
		# ~ print(vel2[1] + " , " + vel2[3] + " , " + vel2[5])
		vel_x = float(vel2[1]) 
		vel_y = float(vel2[3]) 
		vel_z = float(vel2[5]) 
		# ~ print (vel_x)
		
		#get Postion data
		pos1 = str(position)	
		pos2 = pos1.replace(', ',' ').split(' ')
		# ~ print(pos2[1] + " , " + pos2[3] + " , " + pos2[5])
		pos_x = -float(pos2[5]) 
		pos_y = -float(pos2[1]) 
		pos_z = float(pos2[3]) 
		# ~ print(pos1)
		# ~ print(str(pos_x) + " , " + str(pos_y))
		# ~ print (pos_x)
		
		#get Acceleration data
		acc1 = str(acceleration)	
		acc2 = acc1.replace(', ',' ').split(' ')
		# ~ print(acc2[1] + " , " + acc2[3] + " , " + acc2[5])
		acc_x = float(acc2[1]) 
		acc_y = float(acc2[3]) 
		acc_z = float(acc2[5]) 
		# ~ print(acc1)
		# ~ print (acc_x)

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
		
		print(theta_rs)
		# ~ print(theta2)
		# ~ robot.move(0,0,-0.5)




## Ctrl + c to stop robot
except KeyboardInterrupt:
	# ~ print('Done')
	# ~ print(vel2[10])
	# ~ pipe.stop()  
	robot.stop()  
	print('\n\n		Stop!!! See you again!')
