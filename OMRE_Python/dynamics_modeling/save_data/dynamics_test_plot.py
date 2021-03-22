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
v1 = []
v2 = []
v3 = []

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
		
		c_rpm1.append(float(row[8]))
		c_rpm2.append(float(row[9]))
		c_rpm3.append(float(row[10]))
		
		vx.append(float(row[11]))
		vy.append(float(row[12]))
		vw.append(float(row[13]))
		
		v1.append(float(row[14]))
		v2.append(float(row[15]))
		v3.append(float(row[16]))
		
		t.append(float(row[17])-t0)
		
global alpha 
global delta_t
global tau

delta_t = t[1]-t[0]
tau = 2
alpha = 0.5 / tau


############################	filter rpm1
output_v1 = [None] * len(v1)
outputv1n = v1[0]
for n in range(0,len(v1)):
	outputv1n += alpha * (v1[n]-outputv1n)
	output_v1[n]= outputv1n

############################	filter rpm2
output_v2 = [None] * len(v2)
outputv2n = v2[0]
for n in range(0,len(v2)):
	outputv2n += alpha * (v2[n]-outputv2n)
	output_v2[n]= outputv2n

############################	filter rpm3
output_v3 = [None] * len(v3)
outputv3n = v3[0]
for n in range(0,len(v3)):
	outputv3n += alpha * (v3[n]-outputv3n)
	output_v3[n]= outputv3n	

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
np_v1 = np.array(v1).reshape((-1, 1))
np_v2 = np.array(v2).reshape((-1, 1))
np_v3 = np.array(v3).reshape((-1, 1))
np_outputAll = np.hstack((np_posex,np_posey,np_poseTheta,np_posx,np_posy,np_rpm1,np_rpm2,np_rpm3,np_c_rpm1,np_c_rpm2,np_c_rpm3,np_vx,np_vy,np_vw,np_v1,np_v2,np_v3,np_t))
np.savetxt(my_txt1+'_fixed.txt', np_outputAll, delimiter=',',fmt='%2.4f')


nnp_t = np.array(t).reshape((-1, 1))
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
np_v1 = np.array(output_v1).reshape((-1, 1))
np_v2 = np.array(output_v2).reshape((-1, 1))
np_v3 = np.array(output_v3).reshape((-1, 1))
np_outputAll = np.hstack((np_posex,np_posey,np_poseTheta,np_posx,np_posy,np_rpm1,np_rpm2,np_rpm3,np_c_rpm1,np_c_rpm2,np_c_rpm3,np_vx,np_vy,np_vw,np_v1,np_v2,np_v3,np_t))
np.savetxt(my_txt1+'_filter_fixed.txt', np_outputAll, delimiter=',',fmt='%2.4f')



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
plt.ylim(-4,4)
plt.xlim(-4,4)
plt.grid(True)
plt.legend(loc=4,fontsize=8)

#Vrobot
ax2 = plt.subplot(222)
# ~ ax2.plot(t,vx, 'b', label='V Robot', linewidth=3)
ax2.plot(t,vy, 'b', label='V Robot', linewidth=3)
# ~ ax2.plot(t,vw, 'b', label='V Robot', linewidth=3)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('m/s')
ax2.set_title('V Robot', fontsize=12)
plt.grid(True)
plt.legend(loc=4,fontsize=8)

#WheelV
ax3 = plt.subplot(223)
ax3.plot(t,v1, 'r', label='RPM1', linewidth=3)
ax3.plot(t,v2, 'b', label='RPM2', linewidth=3)
ax3.plot(t,v3, 'g', label='RPM3', linewidth=3)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Vol')
ax3.set_title('Vol Wheels', fontsize=12)
plt.grid(True)
plt.legend(loc=7,fontsize=8)

#motor_pwm
ax4 = plt.subplot(224)
ax4.plot(t,c_rpm1, 'r', label='M1 PWM', linewidth=3)
ax4.plot(t,c_rpm2, 'b', label='M2 PWM', linewidth=3)
ax4.plot(t,c_rpm3, 'g', label='M3 PWM', linewidth=3)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('PWM')
ax4.set_title('Motors PWM', fontsize=12)
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
