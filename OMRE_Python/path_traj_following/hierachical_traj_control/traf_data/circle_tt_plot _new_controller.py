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
xd = []
yd = []
zd = []
rs1 = []
rs2 = []
rpm1 =[]
rpm2 =[]
rpm3 =[]
c_rpm1 = []
c_rpm2 = []
c_rpm3 = []
c_v1 = []
c_v2 = []
c_v3 = []
v1 = []
v2 = []
v3 = []

vx= []
vy= []
vw = []
t = []

with open(root.filename,'r') as csvfile:
	plots = csv.reader(csvfile, delimiter=',')
	for idx, row in enumerate(plots):
		if idx == 0:
			t0 = float(row[23])
		x.append(float(row[0]))
		y.append(float(row[1]))
		z.append(float(row[2]))
		
		rs1.append(float(row[3]))
		rs2.append(float(row[4]))
		
		rpm1.append(float(row[5]))
		rpm2.append(float(row[6]))
		rpm3.append(float(row[7]))
		
		c_rpm1.append(float(row[8]))
		c_rpm2.append(float(row[9]))
		c_rpm3.append(float(row[10]))
		
		vx.append(float(row[11]))
		vy.append(float(row[12]))
		vw.append(float(row[13]))
		
		xd.append(float(row[14]))
		yd.append(float(row[15]))
		zd.append(float(row[16]))
		
		v1.append(float(row[17]))
		v2.append(float(row[18]))
		v3.append(float(row[19]))
		
		c_v1.append(float(row[20]))
		c_v2.append(float(row[21]))
		c_v3.append(float(row[22]))
		
		t.append(float(row[23])-t0)

np_t = np.array(t).reshape((-1, 1))
np_posex = np.array(x).reshape((-1, 1))
np_posey = np.array(y).reshape((-1, 1))
np_poseTheta = np.array(z).reshape((-1, 1))
np_posx = np.array(rs1).reshape((-1, 1))
np_posy = np.array(rs2).reshape((-1, 1))
np_rpm1 = np.array(rpm1).reshape((-1, 1))
np_rpm2 = np.array(rpm2).reshape((-1, 1))
np_rpm3 = np.array(rpm3).reshape((-1, 1))
np_c_rpm1 = np.array(c_rpm1).reshape((-1, 1))
np_c_rpm2 = np.array(c_rpm2).reshape((-1, 1))
np_c_rpm3 = np.array(c_rpm3).reshape((-1, 1))
np_vx = np.array(vx).reshape((-1, 1))
np_vy = np.array(vy).reshape((-1, 1))
np_vw = np.array(vw).reshape((-1, 1))
np_xd = np.array(xd).reshape((-1, 1))
np_yd = np.array(yd).reshape((-1, 1))
np_zd = np.array(zd).reshape((-1, 1))

np_v1 = np.array(v1).reshape((-1, 1))
np_v2 = np.array(v2).reshape((-1, 1))
np_v3 = np.array(v3).reshape((-1, 1))

np_c_v1 = np.array(c_v1).reshape((-1, 1))
np_c_v2 = np.array(c_v2).reshape((-1, 1))
np_c_v3 = np.array(c_v3).reshape((-1, 1))
np_outputAll = np.hstack((np_posex,np_posey,np_poseTheta,np_posx,np_posy,np_rpm1,np_rpm2,np_rpm3,np_c_rpm1,np_c_rpm2,np_c_rpm3,np_vx,np_vy,np_vw,np_xd,np_yd,np_zd,np_v1,np_v2,np_v3,np_c_v1,np_c_v2,np_c_v3,np_t))
np.savetxt(my_txt1+'_fixed.txt', np_outputAll, delimiter=',',fmt='%2.4f')


#circle path
theta = np.linspace(0, 2*np.pi, 100)
b = 0.8
r = b
x1 = r*np.cos(theta) 
x2 = r*np.sin(theta)
test_t = 30

fig = plt.figure()
fig.suptitle(my_txt1,fontsize=16)
ax1 = fig.add_subplot(331)
ax2 = fig.add_subplot(332)
ax3 = fig.add_subplot(333)
ax4 = fig.add_subplot(334)
ax5 = fig.add_subplot(335)
ax6 = fig.add_subplot(336)
ax7 = fig.add_subplot(337)
ax8 = fig.add_subplot(338)
ax9 = fig.add_subplot(339)

#Circle
ax1 = plt.subplot(331)
ax1.plot(x,y, 'b', label='Encoder', linewidth=3)
# ~ ax1.plot(rs1,rs2,'g',label='RealSense', linewidth=3)
ax1.plot(x1,x2, 'r--', label='Circle', linewidth=1.5)
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_title('OMRE_Path'+'_R_'+str(r), fontsize=12)
plt.grid(True)
plt.legend()

# ~ #pos_x
ax2 = plt.subplot(332)
ax2.plot(t, x, 'm', label='OMRE', linewidth=3)
ax2.plot(t, xd, 'r--', label='Goal', linewidth=1.5)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('m')
ax2.set_title('X[m]', fontsize=12)
ax2.set_xlim(0, test_t)
ax2.set_ylim(-1, 1)
plt.grid(True)
plt.legend()

#pos_y
ax3 = plt.subplot(333)
ax3.plot(t,y, 'g', label='OMRE', linewidth=3)
ax3.plot(t,yd, 'r--', label='Goal', linewidth=1.5)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('m')
ax3.set_title('Y[m]', fontsize=12)
ax3.set_xlim(0, test_t)
ax3.set_ylim(-1, 1)
plt.grid(True)
plt.legend()

#motor1
ax4 = plt.subplot(334)
ax4.plot(t,rpm1, 'm', label='RPM1', linewidth=3)
ax4.plot(t,c_rpm1, 'r--', label='Commanded RPM', linewidth=1.5)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('RPM')
ax4.set_title('Motor1', fontsize=12)
plt.grid(True)
plt.legend()

#motor2
ax5 = plt.subplot(335)
ax5.plot(t,rpm2, 'g', label='RPM2', linewidth=3)
ax5.plot(t,c_rpm2, 'r--', label='Commanded RPM', linewidth=1.5)
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('RPM')
ax5.set_title('Motor2', fontsize=12)
plt.grid(True)
plt.legend()

#motor3
ax6 = plt.subplot(336)
ax6.plot(t,rpm3, 'c', label='RPM3', linewidth=3)
ax6.plot(t,c_rpm3, 'r--', label='Commanded RPM', linewidth=1.5)
ax6.set_xlabel('Time (s)')
ax6.set_ylabel('RPM')
ax6.set_title('Motor3', fontsize=12)
plt.grid(True)
plt.legend()

#motor1
ax7 = plt.subplot(337)
ax7.plot(t,v1, 'm', label='RPM1', linewidth=3)
ax7.plot(t,c_v1, 'r--', label='Commanded Vol', linewidth=1.5)
ax7.set_xlabel('Time (s)')
ax7.set_ylabel('vol')
ax7.set_title('Motor1', fontsize=12)
plt.grid(True)
plt.legend()

#motor2
ax8 = plt.subplot(338)
ax8.plot(t,v2, 'g', label='RPM2', linewidth=3)
ax8.plot(t,c_v2, 'r--', label='Commanded Vol', linewidth=1.5)
ax8.set_xlabel('Time (s)')
ax8.set_ylabel('vol')
ax8.set_title('Motor2', fontsize=12)
plt.grid(True)
plt.legend()

#motor3
ax9 = plt.subplot(339)
ax9.plot(t,v3, 'c', label='RPM3', linewidth=3)
ax9.plot(t,c_v3, 'r--', label='Commanded Vol', linewidth=1.5)
ax9.set_xlabel('Time (s)')
ax9.set_ylabel('vol')
ax9.set_title('Motor3', fontsize=12)
plt.grid(True)
plt.legend()


plt.subplots_adjust(left=0.15, wspace=0.4, hspace = 0.4, top=0.85)

#Save image into full size 
fig = plt.gcf()
fig.set_size_inches((10, 8), forward=False)
fig.savefig(my_txt1+'.png', dpi=100)

#will not stop the figure if use this
plt.show()
plt.draw()

#the figure will be closed after 3s
plt.show(block=False)
# ~ plt.draw()
# ~ time.sleep(3)

plt.close()
