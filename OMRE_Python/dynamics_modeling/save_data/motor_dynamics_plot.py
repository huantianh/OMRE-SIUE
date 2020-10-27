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
c = []
v = []
r = []
p = []
v_filter = []
c_filter = []

with open(root.filename,'r') as csvfile:
	plots = csv.reader(csvfile, delimiter=',')
	for idx, row in enumerate(plots):
		if idx == 0:
			x0 = float(row[0])
		t.append(float(row[0])-x0)
		c.append(float(row[1]))
		v.append(float(row[2]))
		r.append(float(row[3]))
		p.append(float(row[4]))


global alpha 
global delta_t
global tau

delta_t = t[1]-t[0]
tau = 2
alpha = 0.2 / tau


############################	filter Voltage
inputv = v
outputv = [None] * len(v)
outputvn = v[0]
for n in range(0,len(v)):
	outputvn += alpha * (v[n]-outputvn)
	outputv[n]= outputvn

############################	filter Current
inputc = c
outputc = [None] * len(c)
outputcn = c[0]

for n in range(0,len(c)):
	outputcn += alpha * (c[n]-outputcn)
	outputc[n]= outputcn

np_outputv = np.array(outputv).reshape((-1, 1))
np_outputc = np.array(outputc).reshape((-1, 1))
np_t = np.array(t).reshape((-1, 1))
np_r = np.array(r).reshape((-1, 1))
np_p = np.array(p).reshape((-1, 1))
np_outputAll = np.hstack((np_t,np_outputc,np_outputv,np_r,np_p))
np.savetxt('Motor1_dynamics_filter.txt', np_outputAll, delimiter=',',fmt='%2.4f')


fig = plt.figure()
fig.suptitle(my_txt1,fontsize=16)
ax1 = fig.add_subplot(131)
ax2 = fig.add_subplot(132)
ax3 = fig.add_subplot(133)


#PWM-RPM
ax1 = plt.subplot(131)
ax1.plot(t,p, 'r', label='PWM', linewidth=3)
ax1.plot(t,r, 'b', label='RPM', linewidth=3)
ax1.set_xlabel('time')
ax1.set_ylabel('RPM')
ax1.set_title('PWM_RPM', fontsize=12)
plt.grid(True)
plt.legend(loc=8,fontsize=8)

#PWM-Voltage
ax1 = plt.subplot(132)
# ~ ax1.plot(t,outputv, 'r', label='filter Voltage', linewidth=3)
ax1.plot(t,v, 'b', label='Voltage', linewidth=3)
ax1.set_xlabel('time')
ax1.set_ylabel('Voltage')
ax1.set_title('PWM_Voltage', fontsize=12)
plt.grid(True)
plt.legend(loc=8,fontsize=8)

#PWM-current
ax1 = plt.subplot(133)
# ~ ax1.plot(t,outputc, 'r', label='filter Current', linewidth=3)
ax1.plot(t,c, 'b', label='Current', linewidth=3)
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
