import matplotlib.pyplot as plt
import csv
import numpy as np 
import os, time

#use Tkinter to open a file which we want
from Tkinter import *
import Tkinter, Tkconstants, tkFileDialog

#open the file that we want 
cwd = os.getcwd()
root = Tk()
root.filename = tkFileDialog.askopenfilename(initialdir = cwd,title = "Select file",filetypes = (("txt files","*.txt"),("jpeg files","*.jpg"),("all files","*.*")))
f = open(root.filename, "r")
root.withdraw()

#read that file
my_txt1 = root.filename.split('/')[-1].split('.txt')[0]
text = f.read()

#plotting
t = []
c1 = []
c2 = []
c3 = []
v1 = []
v2 = []
v3 = []
r1 = []
r2 = []
r3 = []
p = []
v_filter1 = []
v_filter2 = []
v_filter3 = []
c_filter1 = []
c_filter2 = []
c_filter3 = []

with open(root.filename,'r') as csvfile:
	plots = csv.reader(csvfile, delimiter=',')
	for idx, row in enumerate(plots):
		if idx == 0:
			x0 = float(row[0])
		t.append(float(row[0])-x0)
		c1.append(float(row[1]))
		c2.append(float(row[2]))
		c3.append(float(row[3]))
		v1.append(float(row[4]))
		v2.append(float(row[5]))
		v3.append(float(row[6]))
		r1.append(float(row[7]))
		r2.append(float(row[8]))
		r3.append(float(row[9]))
		p.append(float(row[10]))


global alpha 
global delta_t
global tau

delta_t = t[1]-t[0]
tau = 2
alpha = 0.2 / tau


############################	filter Voltage
inputv1 = v1
outputv1 = [None] * len(v1)
outputvn1 = v1[0]
for n in range(0,len(v1)):
	outputvn1 += alpha * (v1[n]-outputvn1)
	outputv1[n]= outputvn1

inputv2 = v2
outputv2 = [None] * len(v2)
outputvn2 = v2[0]
for n in range(0,len(v2)):
	outputvn2 += alpha * (v2[n]-outputvn2)
	outputv2[n]= outputvn2

inputv3 = v3
outputv3 = [None] * len(v3)
outputvn3 = v3[0]
for n in range(0,len(v3)):
	outputvn3 += alpha * (v3[n]-outputvn3)
	outputv3[n]= outputvn3	


############################	filter Current
inputc1 = c1
outputc1 = [None] * len(c1)
outputcn1 = c1[0]
for n in range(0,len(c1)):
	outputcn1 += alpha * (c1[n]-outputcn1)
	outputc1[n]= outputcn1

inputc2 = c2
outputc2 = [None] * len(c2)
outputcn2 = c2[0]
for n in range(0,len(c2)):
	outputcn2 += alpha * (c2[n]-outputcn2)
	outputc2[n]= outputcn2

inputc3 = c3
outputc3 = [None] * len(c3)
outputcn3 = c3[0]
for n in range(0,len(c3)):
	outputcn3 += alpha * (c3[n]-outputcn3)
	outputc3[n]= outputcn3

np_outputv1 = np.array(outputv1).reshape((-1, 1))
np_outputv2 = np.array(outputv2).reshape((-1, 1))
np_outputv3 = np.array(outputv3).reshape((-1, 1))
np_outputc1 = np.array(outputc1).reshape((-1, 1))
np_outputc2 = np.array(outputc2).reshape((-1, 1))
np_outputc3 = np.array(outputc3).reshape((-1, 1))
np_t = np.array(t).reshape((-1, 1))
np_r1 = np.array(r1).reshape((-1, 1))
np_r2 = np.array(r2).reshape((-1, 1))
np_r3 = np.array(r3).reshape((-1, 1))
np_p = np.array(p).reshape((-1, 1))
np_outputAll = np.hstack((np_t,np_outputc1,np_outputc2,np_outputc3,np_outputv1,np_outputv2,np_outputv3,np_r1,np_r2,np_r3,np_p))
np.savetxt('Motor_dynamics_filter.txt', np_outputAll, delimiter=',',fmt='%2.4f')

np_t = np.array(t).reshape((-1, 1))
np_c1 = np.array(c1).reshape((-1, 1))
np_c2 = np.array(c2).reshape((-1, 1))
np_c3 = np.array(c3).reshape((-1, 1))
np_v1 = np.array(v1).reshape((-1, 1))
np_v2 = np.array(v2).reshape((-1, 1))
np_v3 = np.array(v3).reshape((-1, 1))
np_r1 = np.array(r1).reshape((-1, 1))
np_r2 = np.array(r2).reshape((-1, 1))
np_r3 = np.array(r3).reshape((-1, 1))
np_p = np.array(p).reshape((-1, 1))
np_outputAll = np.hstack((np_t,np_c1,np_c2,np_c3,np_v1,np_v2,np_v3,np_r1,np_r2,np_r3,np_p))
np.savetxt(my_txt1+'_fixed.txt', np_outputAll, delimiter=',',fmt='%2.4f')




fig = plt.figure()
fig.suptitle(my_txt1,fontsize=16)
ax1 = fig.add_subplot(131)
ax2 = fig.add_subplot(132)
ax3 = fig.add_subplot(133)


#PWM-RPM
ax1 = plt.subplot(131)
ax1.plot(t,p, 'r', label='PWM', linewidth=3)
ax1.plot(t,r1, 'b', label='RPM1', linewidth=3)
ax1.plot(t,r2, 'g', label='RPM2', linewidth=3)
ax1.plot(t,r3, 'k', label='RPM3', linewidth=3)
ax1.set_xlabel('time')
ax1.set_ylabel('RPM')
ax1.set_title('PWM_RPM', fontsize=12)
plt.grid(True)
plt.legend(loc=8,fontsize=8)

#PWM-Voltage
ax1 = plt.subplot(132)
# ~ ax1.plot(t,outputv, 'r', label='filter Voltage', linewidth=3)
ax1.plot(t,v1, 'b', label='Voltage1', linewidth=3)
ax1.plot(t,v2, 'g', label='Voltage2', linewidth=3)
ax1.plot(t,v3, 'k', label='Voltage3', linewidth=3)
ax1.set_xlabel('time')
ax1.set_ylabel('Voltage')
ax1.set_title('PWM_Voltage', fontsize=12)
plt.grid(True)
plt.legend(loc=8,fontsize=8)

#PWM-current
ax1 = plt.subplot(133)
# ~ ax1.plot(t,outputc, 'r', label='filter Current', linewidth=3)
ax1.plot(t,c1, 'b', label='Current1', linewidth=3)
ax1.plot(t,c2, 'g', label='Current2', linewidth=3)
ax1.plot(t,c3, 'k', label='Current3', linewidth=3)
ax1.set_xlabel('time')
ax1.set_ylabel('Ampe')
ax1.set_title('PWM_Current', fontsize=12)
plt.grid(True)
plt.legend(loc=8,fontsize=8)


plt.subplots_adjust(left=0.15, wspace=0.4, hspace = 0.4, top=0.85)

#Save image into full size 
fig = plt.gcf()
fig.set_size_inches((15, 5), forward=False)
fig.savefig(my_txt1+'.png', dpi=100)

#will not stop the figure if use this
plt.show()
plt.draw()

#the figure will be closed after 3s
plt.show(block=False)
# ~ plt.draw()
# ~ time.sleep(3)

plt.close()
