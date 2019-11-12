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
vx= []
vy= []
v = []
t = []

with open(root.filename,'r') as csvfile:
	plots = csv.reader(csvfile, delimiter=',')
	for idx, row in enumerate(plots):
		if idx == 0:
			t0 = float(row[11])
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
		
		t.append(float(row[11])-t0)

x1 = 0.1
y1 = 0.1

fig = plt.figure()
fig.suptitle(my_txt1,fontsize=16)
ax1 = fig.add_subplot(221)
ax2 = fig.add_subplot(222)
ax3 = fig.add_subplot(223)
ax4 = fig.add_subplot(224)

#Line
ax1 = plt.subplot(221)
ax1.plot(x,y, 'b', label='Encoder', linewidth=3)
ax1.plot(rs1,rs2,'g',label='RealSense', linewidth=3)
ax1.plot(x1,y1, 'r--', label='Line', linewidth=1.5)
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_title('OMRE_Path', fontsize=12)
plt.grid(True)
plt.legend()

#motor1
ax2 = plt.subplot(222)
ax2.plot(t,vx, 'm', label='Vx', linewidth=3)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('m/s')
ax2.set_title('Vx', fontsize=12)
plt.grid(True)
plt.legend()

#motor2
ax3 = plt.subplot(223)
ax3.plot(t,vy, 'g', label='Vy', linewidth=3)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('m/s')
ax3.set_title('Vy', fontsize=12)
plt.grid(True)
plt.legend()

#motor3
ax4 = plt.subplot(224)
ax4.plot(t,v, 'c', label='V', linewidth=3)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('m/s')
ax4.set_title('V', fontsize=12)
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