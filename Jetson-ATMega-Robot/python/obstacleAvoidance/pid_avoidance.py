import libomni as robot  #Library tha handles all the serial commands to arduino AtMega
import time


robot.enablePID(1)
us = [0,0,0,0,0]



#  leftleft,left,center,right,rightright
us = [0,0,0,0,0]
alreadyForward = False
count = 0

while True:

	for x in range(5):
		us[x] = robot.ultrasound(x+1)
	
	for x in range(5):
		print("Ultrasound "+str(x)+": "+str(us[x]))
	print(count)
	if(count >=50):
		robot.move(-.4,0,0)
		time.sleep(.7)
		robot.move(0,0,2)
		time.sleep(.7)
		count=0
	if(us[2] <=15):
		if(us[1] < us[3]):
			robot.move(0,0,-2)
		else:
			robot.move(0,0,2)
		count +=1 
	elif(us[1] <= 29):
		robot.move(0,0,-2)
		count +=1 
	elif(us[3] <=29):
		robot.move(0,0,2)
		count +=1 
	elif(us[0] <= 10):
		robot.move(0,0,-2)
		count +=1 
	elif(us[4] <= 10):
		robot.move(0,0,2)
		count +=1 
	else:
		robot.move(.4,0,0)
		if(count>4):
			count -=4
		else:
			count = 0


	

	
#forward(2,60)
#turn(.5,70)
