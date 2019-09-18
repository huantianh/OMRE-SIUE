# First import the library
import pyrealsense2 as rs
import time

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
		print(str(pos_x) + " , " + str(pos_y))
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


## Ctrl + c to stop robot
except KeyboardInterrupt:
	# ~ print('Done')
	# ~ print(vel2[10])
	pipe.stop()    
	print('\n\n		Stop!!! See you again!')
