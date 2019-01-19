import time
from minIMU import minIMU


imu = minIMU()
imu.enable()

#start_time = time.time()
#pi_time = time.time() - start_time

imudata = [0,0,0,0]
[ax, ay, az, wx, wy, wz] = imu.getIMURaw()

while True:
	imudata.append([ax, ay, az, wx, wy, wz])
	print(imu.getIMURaw())
	#time.sleep (0.1)
	imufile = open ( "imu_data.txt" , "w")
	imufile.writelines(["%s\n" % item for item in imudata])
	imufile.close()
