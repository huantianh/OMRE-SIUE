import libomni as robot  #Library tha handles all the serial commands to arduino AtMega
import time


robot.enablePID(0)
robot.motors(0,0,0)
time.sleep(.5)

#positive is right negative is left
#if given 0 seconds will continue forever
def turn(seconds,speed):
	robot.motors(speed,speed,speed)
	if(seconds != 0):
		time.sleep(seconds)
		robot.motors(0,0,0)

#positive forward backwards negative
#if given 0 seconds will continue forever
def forward(seconds,speed):
	robot.motors(0,speed,-speed)
	if(seconds != 0):
		time.sleep(seconds)
		robot.motors(0,0,0)

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
		forward(1,-65)
		turn(.6,60)
		count=0
	if(us[2] <=15):
		turn(0,55)
		count +=1 
	elif(us[1] <= 29):
		turn(0,55)
		count +=1 
	elif(us[3] <=29):
		turn(0,-55)
		count +=1 
	elif(us[0] <= 10):
		turn(0,55)
		count +=1 
	elif(us[4] <= 10):
		turn(0,-55)
		count +=1 
	else:
		forward(0,60)
		if(count>4):
			count -=4
		else:
			count = 0


	
#forward(2,60)
#turn(.5,70)
