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


def filter(x):
	y = [None] * len(x)
	for n in range(0,len(x)):
		a = 30
		T = 0.1
		b = -a*T
		y[n] = np.exp(b)*x[n-1]+x[n]
	return y	

input = rpm1
print(input)		
output = filter(rpm1)
print(output)

fig = plt.figure()
fig.suptitle("Motor1",fontsize=16)
ax1 = fig.add_subplot(221)

#motor1
ax1 = plt.subplot(221)
ax1.plot(t,input, 'r--', label='RPM1', linewidth=1.5)
ax1.plot(t,output, 'm', label='RPM1_f', linewidth=2)
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('RPM')
ax1.set_title('Motor1', fontsize=12)
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
