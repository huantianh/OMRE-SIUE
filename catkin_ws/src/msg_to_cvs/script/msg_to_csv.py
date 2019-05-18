#!/usr/bin/env python

import rospy
import csv
from std_msgs.msg import Int32MultiArray

def callback(msg):
	nowTime = rospy.Time.now()
	changeTime = nowTime - startTime
	with open('test.csv', 'ab') as csvfile:
		filewriter = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
		filewriter.writerow([str(changeTime.secs) + "." + str(changeTime.nsecs),str(msg.data[0]),
							 str(msg.data[1]),str(msg.data[2])])
		
rospy.init_node('csv_converter')
startTime = rospy.Time.now()
with open('test.csv', 'wb') as csvfile:
	filewriter = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
	filewriter.writerow(['Time','RPM1','RPM2','RPM3','Input'])
	
if __name__ == '__main__':
	rospy.Subscriber("/RPM", Int32MultiArray, callback)
	rospy.spin()
