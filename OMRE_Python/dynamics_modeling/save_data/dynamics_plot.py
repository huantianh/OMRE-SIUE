import matplotlib.pyplot as plt
import csv
import numpy as np 
import os, time

from scipy import stats

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
vw = []

Fv= []
Fvn = []
R_f = []
t = []

n = []

cur1 = []
cur2 = []
cur3 = []
i1 = []
i2 = []
i3 = []

with open(root.filename,'r') as csvfile:
	plots = csv.reader(csvfile, delimiter=',')
	for idx, row in enumerate(plots):
		if idx == 0:
			t0 = float(row[17])
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
		vw.append(float(row[10]))
		
		m1_cur.append(float(row[11]))
		m2_cur.append(float(row[12]))
		m3_cur.append(float(row[13]))
		
		Fv.append(float(row[14]))
		Fvn.append(float(row[15]))
		R_f.append(float(row[16]))
		
		t.append(float(row[17])-t0)



fig = plt.figure()
fig.suptitle(my_txt1,fontsize=16)
ax1 = fig.add_subplot(221)
ax2 = fig.add_subplot(222)
ax3 = fig.add_subplot(223)
ax4 = fig.add_subplot(224)
# ~ ax5 = fig.add_subplot(335)

#Path
ax1 = plt.subplot(221)
# ~ ax1.plot(x,y, 'b', label='Encoder', linewidth=3)
ax1.plot(rs1,rs2,'g',label='RealSense', linewidth=3)
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_title('OMRE_Path', fontsize=12)
# ~ plt.ylim(-2,2)
plt.grid(True)
plt.legend(loc=4,fontsize=8)

#Vrobot
ax2 = plt.subplot(222)
ax2.plot(t,vw, 'b', label='V Robot', linewidth=3)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('m/s')
ax2.set_title('V Robot', fontsize=12)
plt.grid(True)
plt.legend(loc=4,fontsize=8)

#WheelV
ax3 = plt.subplot(223)
ax3.plot(t,rpm1, 'r', label='RPM1', linewidth=3)
ax3.plot(t,rpm2, 'b', label='RPM2', linewidth=3)
ax3.plot(t,rpm3, 'g', label='RPM3', linewidth=3)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('RPM')
ax3.set_title('V Wheels', fontsize=12)
plt.grid(True)
plt.legend(loc=7,fontsize=8)

#motor_current
ax4 = plt.subplot(224)
ax4.plot(t,m1_cur, 'r', label='M1 Current', linewidth=3)
ax4.plot(t,m2_cur, 'b', label='M2 Current', linewidth=3)
ax4.plot(t,m3_cur, 'g', label='M3 Current', linewidth=3)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('A')
ax4.set_title('Motors Current', fontsize=12)
plt.grid(True)
plt.legend(loc=7,fontsize=8)

# ~ #Fv
# ~ ax5 = plt.subplot(335)
# ~ ax5.plot(vx,Fv, 'r', label='Fv', linewidth=3)
# ~ ax5.set_xlabel('vx(m/s)')
# ~ ax5.set_ylabel('Fv(N)')
# ~ ax5.set_title('Traction Forces Fv', fontsize=12)
# ~ plt.grid(True)
# ~ plt.legend(loc=7,fontsize=8)

# ~ #Fvn
# ~ ax6 = plt.subplot(336)
# ~ ax6.plot(vy,Fvn, 'r', label='Fv', linewidth=3)
# ~ ax6.set_xlabel('vy(m/s)')
# ~ ax6.set_ylabel('Fvn(N)')
# ~ ax6.set_title('Traction Forces Fvn', fontsize=12)
# ~ plt.grid(True)
# ~ plt.legend(loc=7,fontsize=8)


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
