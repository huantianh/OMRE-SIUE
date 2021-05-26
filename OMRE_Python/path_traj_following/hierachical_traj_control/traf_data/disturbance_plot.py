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

ax = []
ay = []
aw = []

rpm1 =[]
rpm2 =[]
rpm3 =[]

pwm1 = []
pwm2 = []
pwm3 = []

vol1 = []
vol2 = []
vol3 = []

cur1 = []
cur2 = []
cur3 = []

vx= []
vy= []
vw = []
t = []

with open(root.filename,'r') as csvfile:
	plots = csv.reader(csvfile, delimiter=',')
	for idx, row in enumerate(plots):
		if idx == 0:
			t0 = float(row[21])
		rs1.append(float(row[0]))
		rs2.append(float(row[1]))
		z.append(float(row[2]))
		
		vx.append(float(row[3]))
		vy.append(float(row[4]))
		vw.append(float(row[5]))
		
		ax.append(float(row[6]))
		ay.append(float(row[7]))
		aw.append(float(row[8]))
		
		pwm1.append(float(row[9]))
		pwm2.append(float(row[10]))
		pwm3.append(float(row[11]))
		
		rpm1.append(float(row[12]))
		rpm2.append(float(row[13]))
		rpm3.append(float(row[14]))
		
		vol1.append(float(row[15]))
		vol2.append(float(row[16]))
		vol3.append(float(row[17]))
		
		cur1.append(float(row[18]))
		cur2.append(float(row[19]))
		cur3.append(float(row[20]))
		
		t.append(float(row[21])-t0)

np_t = np.array(t).reshape((-1, 1))

np_posx = np.array(rs1).reshape((-1, 1))
np_posy = np.array(rs2).reshape((-1, 1))
np_poseTheta = np.array(z).reshape((-1, 1))

np_vx = np.array(vx).reshape((-1, 1))
np_vy = np.array(vy).reshape((-1, 1))
np_vw = np.array(vw).reshape((-1, 1))

np_ax = np.array(ax).reshape((-1, 1))
np_ay = np.array(ay).reshape((-1, 1))
np_aw = np.array(aw).reshape((-1, 1))


np_pwm1 = np.array(pwm1).reshape((-1, 1))
np_pwm2 = np.array(pwm2).reshape((-1, 1))
np_pwm3 = np.array(pwm3).reshape((-1, 1))

np_rpm1 = np.array(rpm1).reshape((-1, 1))
np_rpm2 = np.array(rpm2).reshape((-1, 1))
np_rpm3 = np.array(rpm3).reshape((-1, 1))

np_vol1 = np.array(vol1).reshape((-1, 1))
np_vol2 = np.array(vol2).reshape((-1, 1))
np_vol3 = np.array(vol3).reshape((-1, 1))

np_cur1 = np.array(cur1).reshape((-1, 1))
np_cur2 = np.array(cur2).reshape((-1, 1))
np_cur3 = np.array(cur3).reshape((-1, 1))

np_outputAll = np.hstack((np_posx,np_posy,np_poseTheta,np_vx,np_vy,np_vw,np_ax,np_ay,np_aw,np_pwm1,np_pwm2,np_pwm3,np_rpm1,np_rpm2,np_rpm3,np_vol1,np_vol2,np_vol3,np_cur1,np_cur2,np_cur3,np_t))
np.savetxt(my_txt1+'_fixed.txt', np_outputAll, delimiter=',',fmt='%2.4f')



fig = plt.figure()
fig.suptitle(my_txt1,fontsize=16)
ax1 = fig.add_subplot(231)
ax2 = fig.add_subplot(232)
ax3 = fig.add_subplot(233)
ax4 = fig.add_subplot(234)
ax5 = fig.add_subplot(235)
ax6 = fig.add_subplot(236)

#Path
ax1 = plt.subplot(231)
ax1.plot(rs1,rs2,'g',label='RealSense', linewidth=3)
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_title('OMRE_Path', fontsize=12)
plt.grid(True)
plt.legend()

# robot speed
ax2 = plt.subplot(232)
# ~ ax2.plot(t, vx, 'm', label='OMRE', linewidth=3)
# ~ ax2.plot(t, vy, 'm', label='OMRE', linewidth=3)
ax2.plot(t, vy, 'm', label='OMRE', linewidth=3)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('m/s')
ax2.set_title('Robot Speed', fontsize=12)
plt.grid(True)
plt.legend()

# robot acc
ax3 = plt.subplot(233)
ax3.plot(t,ax, 'g', label='m1', linewidth=3)
ax3.plot(t,ay, 'm', label='m2', linewidth=1.5)
ax3.plot(t,aw, 'r', label='m3', linewidth=1.5)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('m2/s')
ax3.set_title('Motor Acc', fontsize=12)
plt.grid(True)
plt.legend()


# rpm
ax4 = plt.subplot(234)
ax4.plot(t,rpm1, 'g', label='RPM1', linewidth=3)
ax4.plot(t,rpm2, 'm', label='RPM2', linewidth=3)
ax4.plot(t,rpm3, 'r', label='RPM3', linewidth=3)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('RPM')
ax4.set_title('Motor RPM', fontsize=12)
plt.grid(True)
plt.legend()

# vol
ax5 = plt.subplot(235)
ax5.plot(t,vol1, 'g', label='vol1', linewidth=3)
ax5.plot(t,vol2, 'm', label='vol2', linewidth=3)
ax5.plot(t,vol3, 'r', label='vol3', linewidth=3)
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Vol')
ax5.set_title('Motor Vol', fontsize=12)
plt.grid(True)
plt.legend()

# current
ax6 = plt.subplot(236)
ax6.plot(t,cur1, 'g', label='Cur1', linewidth=3)
ax6.plot(t,cur2, 'm', label='Cur2', linewidth=3)
ax6.plot(t,cur3, 'r', label='Cur3', linewidth=3)
ax6.set_xlabel('Time (s)')
ax6.set_ylabel('Cur')
ax6.set_title('Motor Cur', fontsize=12)
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
