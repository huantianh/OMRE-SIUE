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
x = []
y = []
z = []
rs1 = []
rs2 = []
rpm1 =[]
rpm2 =[]
rpm3 =[]
c_rpm1 = []
c_rpm2 = []
c_rpm3 = []
vx= []
vy= []
v = []
t = []

global rpm_f

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
	for n in range(1,len(x)):
		y = [None] * len(x)
		y[n] = np.exp(-1*0.1)*x[n-1]+x[n]
	return y	

input = rpm1
print(rpm1)		
output = filter(rpm1)
rpm1_f = []
rpm1_f.append(output)

#circle path
theta = np.linspace(0, 2*np.pi, 100)
b = 1
r = b
x1 = r*np.cos(theta)
# ~ x2 = r*np.sin(theta)+ b
x2 = r*np.sin(theta)

fig = plt.figure()
fig.suptitle(my_txt1,fontsize=16)
ax1 = fig.add_subplot(221)
ax2 = fig.add_subplot(222)
ax3 = fig.add_subplot(223)
ax4 = fig.add_subplot(224)

#motor1
ax2 = plt.subplot(222)
ax2.plot(t,rpm1, 'm', label='RPM1', linewidth=3)
ax2.plot(t,rpm1_f, 'r--', label='Commanded Wd', linewidth=1.5)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('RPM')
ax2.set_title('Motor1', fontsize=12)
plt.grid(True)
plt.legend()

#motor2
ax3 = plt.subplot(223)
ax3.plot(t,rpm2, 'g', label='RPM2', linewidth=3)
ax3.plot(t,c_rpm2, 'r--', label='Commanded Wd', linewidth=1.5)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('RPM')
ax3.set_title('Motor2', fontsize=12)
plt.grid(True)
plt.legend()

#motor3
ax4 = plt.subplot(224)
ax4.plot(t,rpm3, 'c', label='RPM3', linewidth=3)
ax4.plot(t,c_rpm3, 'r--', label='Commanded Wd', linewidth=1.5)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('RPM')
ax4.set_title('Motor3', fontsize=12)
plt.grid(True)
plt.legend()

plt.subplots_adjust(left=0.15, wspace=0.4, hspace = 0.4, top=0.85)

#Save image into full size 
fig = plt.gcf()
fig.set_size_inches((10, 10), forward=False)
fig.savefig(my_txt1+'.png', dpi=100)

#will not stop the figure if use this
plt.show()
plt.draw()

#the figure will be closed after 3s
plt.show(block=False)
# ~ plt.draw()
# ~ time.sleep(3)

plt.close()
