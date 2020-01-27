import matplotlib.pyplot as plt
import csv
import numpy as np 
import os, time

#plotting
rpm1 =[]
rpm2 =[]
rpm3 =[]
c_rpm1 = []
c_rpm2 = []
c_rpm3 = []
t = []

with open("Circle_TraF_K_1.5_step_t_0.1_del_t_0.1.txt",'r') as csvfile:
	plots = csv.reader(csvfile, delimiter=',')
	for idx, row in enumerate(plots):
		if idx == 0:
			t0 = float(row[6])	
		rpm1.append(float(row[0]))
		rpm2.append(float(row[1]))
		rpm3.append(float(row[2]))
		
		c_rpm1.append(float(row[3]))
		c_rpm2.append(float(row[4]))
		c_rpm3.append(float(row[5]))
		
		t.append(float(row[6])-t0)
		
global a 
global T 
global b

a = 0.1
T = 0.1
b = -a*T

############################	filter Motor1
input1 = rpm1
print(input1)	
output1 = [None] * len(rpm1)
for n in range(0,len(rpm1)):
	output1[n] = np.exp(b)*rpm1[n-1]+a*rpm1[n]
print(output1)

############################	filter Motor2
input2 = rpm2
print(input2)	
output2 = [None] * len(rpm2)
for n in range(0,len(rpm2)):
	output2[n] = np.exp(b)*rpm2[n-1]+a*rpm2[n]
print(output2)

############################	filter Motor3
input3 = rpm3
print(input3)	
output3 = [None] * len(rpm3)
for n in range(0,len(rpm3)):
	output3[n] = np.exp(b)*rpm3[n-1]+a*rpm3[n]
print(output3)


############################	plot
fig = plt.figure()
fig.suptitle("RPM_test",fontsize=16)
ax1 = fig.add_subplot(221)

#motor1
ax1 = plt.subplot(221)
ax1.plot(t,input1, 'r--', label='rpm1', linewidth=1.5)
ax1.plot(t,output1, 'm', label='rpm1_f', linewidth=2)
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('RPM')
ax1.set_title('Motor1', fontsize=12)
plt.grid(True)
plt.legend()

#motor2
ax2 = plt.subplot(222)
ax2.plot(t,input2, 'r--', label='rpm2', linewidth=1.5)
ax2.plot(t,output2, 'g', label='rpm2_f', linewidth=2)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('RPM')
ax2.set_title('Motor2', fontsize=12)
plt.grid(True)
plt.legend()

#motor3
ax3 = plt.subplot(223)
ax3.plot(t,input3, 'r--', label='rpm3', linewidth=1.5)
ax3.plot(t,output3, 'b', label='rpm3_f', linewidth=2)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('RPM')
ax3.set_title('Motor3', fontsize=12)
plt.grid(True)
plt.legend()

plt.subplots_adjust(left=0.15, wspace=0.4, hspace = 0.4, top=0.85)

#Save image into full size 
fig = plt.gcf()
fig.set_size_inches((10, 10), forward=False)
fig.savefig("RPM_test"+'.png', dpi=100)

#will not stop the figure if use this
plt.show()
plt.draw()

#the figure will be closed after 3s
plt.show(block=False)
# ~ plt.draw()
# ~ time.sleep(3)

plt.close()
