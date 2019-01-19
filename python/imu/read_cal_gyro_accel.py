from minIMU import minIMU
from time import sleep

imu = minIMU()
imu.enable()

try:
	while True:
		# Read accel/gyro readings
		[ax, ay, az, wx, wy, wz] = imu.getIMUFil()
                
		# Print to screen
                print("ax: %6.2f ay: %6.2f az: %6.2f m/s^2      wx: %6.2f wy: %6.2f wz: %6.2f deg/s") %(ax,ay,az,wx,wy,wz)
                # Wait 10 ms
                sleep(0.01)
                
except KeyboardInterrupt:
        print('')
