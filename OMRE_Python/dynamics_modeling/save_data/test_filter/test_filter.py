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
m1_cur = []
m2_cur = []
m3_cur = []
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

with open(root.filename,'r') as csvfile:
	plots = csv.reader(csvfile, delimiter=',')
	for idx, row in enumerate(plots):
		if idx == 0:
			t0 = float(row[14])
		x.append(float(row[0]))
		y.append(float(row[1]))
		z.append(float(row[2]))
		
		rs1.append(float(row[3]))
		rs2.append(float(row[4]))
		
		rpm1.append(float(row[5]))
		rpm2.append(float(row[6]))
		rpm3.append(float(row[7]))
		
		vx.append(float(row[8]))
		vy.append(float(row[9]))
		v.append(float(row[10]))
		
		m1_cur.append(float(row[11]))
		m2_cur.append(float(row[12]))
		m3_cur.append(float(row[13]))
		
		t.append(float(row[14])-t0)

		
global alpha 
global delta_t
global tau

delta_t = t[1]-t[0]
print delta_t
tau = 2
# ~ alpha = delta_t / tau
alpha = 0.5 / tau

############################	filter V Robot
inputv = v
# ~ print(input1)	
outputv = [None] * len(v)
outputvn = v[0]
for n in range(0,len(rpm1)):
	outputvn += alpha * (v[n]-outputvn)
	outputv[n]=outputvn



############################	filter Motor1
input1 = rpm1
# ~ print(input1)	
outputrpm1 = [None] * len(rpm1)
outputrpm1n = rpm1[0]
for n in range(0,len(rpm1)):
	outputrpm1n += alpha * (rpm1[n]-outputrpm1n)
	outputrpm1[n]=outputrpm1n

############################	filter Motor2
input2 = rpm2
# ~ print(input2)	
outputrpm2 = [None] * len(rpm2)
outputrpm2n = rpm2[0]
for n in range(0,len(rpm2)):
	outputrpm2n += alpha * (rpm2[n]-outputrpm2n)
	outputrpm2[n]=outputrpm2n
# ~ print(outputrpm2)

############################	filter Motor3
input3 = rpm3
# ~ print(input2)	
outputrpm3 = [None] * len(rpm3)
outputrpm3n = rpm3[0]
for n in range(0,len(rpm3)):
	outputrpm3n += alpha * (rpm3[n]-outputrpm3n)
	outputrpm3[n]=outputrpm3n
# ~ print(outputrpm2)


fig = plt.figure()
fig.suptitle(my_txt1+'_Filter',fontsize=16)
ax1 = fig.add_subplot(221)
ax2 = fig.add_subplot(222)
ax3 = fig.add_subplot(223)
ax4 = fig.add_subplot(224)

#Path
ax1 = plt.subplot(221)
ax1.plot(rs1,rs2,'g',label='RealSense', linewidth=3)
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_title('OMRE_Path', fontsize=12)
plt.ylim(-2,2)
plt.grid(True)
plt.legend(loc=4)

#Vrobot
ax2 = plt.subplot(222)
ax2.plot(t,outputv, 'b', label='V Robot', linewidth=3)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('m/s')
ax2.set_title('V Robot', fontsize=12)
plt.grid(True)
plt.legend(loc=4)

#WheelV
ax3 = plt.subplot(223)
ax3.plot(t,outputrpm1, 'r', label='RPM1', linewidth=3)
ax3.plot(t,outputrpm2, 'b', label='RPM2', linewidth=3)
ax3.plot(t,outputrpm3, 'g', label='outputrpm3', linewidth=3)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('RPM')
ax3.set_title('V Wheels', fontsize=12)
plt.grid(True)
plt.legend(loc=7)

#motors current
ax4 = plt.subplot(224)
ax4.plot(t,m1_cur, 'r', label='M1 Current', linewidth=3)
ax4.plot(t,m2_cur, 'b', label='M2 Current', linewidth=3)
ax4.plot(t,m3_cur, 'g', label='M3 Current', linewidth=3)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('A')
ax4.set_title('Motors Current', fontsize=12)
plt.grid(True)
plt.legend(loc=7)


plt.subplots_adjust(left=0.15, wspace=0.4, hspace = 0.4, top=0.85)

#Save image into full size 
fig = plt.gcf()
fig.set_size_inches((10, 10), forward=False)
fig.savefig(my_txt1+'_filter'+'.png', dpi=100)

#will not stop the figure if use this
plt.show()
plt.draw()

#the figure will be closed after 3s
plt.show(block=False)
# ~ plt.draw()
# ~ time.sleep(3)

plt.close()
