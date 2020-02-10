import libomni as robot  #Library that handles all the serial commands to arduino AtMega
import pyrealsense2 as rs
import numpy as np
import time,os,serial,math

#folder where saving all the data
save_folder = "traf_data/"
current_directory = os.getcwd()
save_directory = os.path.join(current_directory, save_folder)

try: 
	while True:
		mode = str(input("Enter s to start "))
		
		if mode == 's':
			while True:
				file = open(save_folder +"RPM_filter_openloop"+".txt","a")
			
				########################################################			Sending input U to Arduino
				robot.motor_pwm(int(100),int(150),int(255))
				
				########################################################		    Getting RPM from Arduino 
				m1_rpm = int(robot.rpm(0))											
				m2_rpm = int(robot.rpm(1))
				m3_rpm = int(robot.rpm(2))
				data_rpm = str(m1_rpm)+' , ' +str(m2_rpm)+ ' , ' +str(m3_rpm)
				print(data_rpm)
		
				########################################################			Recording data
				time_running = time.time()		
				file.writelines(str(m1_rpm)+" , "+str(m2_rpm)+" , "+str(m3_rpm)+" , "+str(time_running)+ "\n")

## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
	robot.stop()    
	#~ file.close() 
	print('\n\n		Stop!!! See you again!')
