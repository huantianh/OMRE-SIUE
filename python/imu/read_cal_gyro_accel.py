from minIMU import minIMU
from time import sleep

imu = minIMU()
imu.enable()

try:
	while True:
		# Read accel/gyro readings
		[ax, ay, az, wx, wy, wz] = imu.getIMUFil()
                
		# Print to screen
                print("%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f") %(ax,ay,az,wx,wy,wz)
                # Wait 10 ms
                sleep(0.01)
                
except KeyboardInterrupt:
        print('')
