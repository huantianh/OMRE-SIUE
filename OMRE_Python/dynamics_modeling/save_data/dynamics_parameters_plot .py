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
v1 = []
v2 = []
v3 = []
vx= []
vy= []
vw = []
fx= []
fy = []
M = []
t = []
w = []

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
		
		v1.append(float(row[8]))
		v2.append(float(row[9]))
		v3.append(float(row[10]))
		
		m1_cur.append(float(row[11]))
		m2_cur.append(float(row[12]))
		m3_cur.append(float(row[13]))
		
		vx.append(float(row[14]))
		vy.append(float(row[15]))
		vw.append(float(row[16]))
		
		t.append(float(row[17])-t0)

##################################################### 		Motor Current
###find indice of the current mean
for i in range(0,len(t)):
	if t[i] > 2 and t[i] < 7:
		n.append(i)

### find motor current mean
################# motor 1 current
for c in n:
	cur1.append(m1_cur[c])
i1 = np.mean(cur1)
# ~ print(i1)
################# motor 2 current
for a in n:
	cur2.append(m2_cur[a])
i2 = np.mean(cur2)
# ~ print(i2)
################# motor 3 current
for b in n:
	cur3.append(m3_cur[b])
i3 = np.mean(cur3)
# ~ print(i3)

###################################################### 		Calculation
li = 46.85                  #reduction of motors
Kt = 0.3535          		#Motor Torque Constant
r = 0.03					#Wheel Radius
l = 0.19                    #distance from wheel to CG

T1 = li*Kt*np.absolute(i1)
T2 = li*Kt*np.absolute(i2)
T3 = li*Kt*np.absolute(i3)

f1 = T1/r
f2 = T2/r
f3 = T3/r

fv  = f2*np.cos(np.pi/6) - f3*np.cos(np.pi/6)
fvn = f1 - f2*np.sin(np.pi/6) - f3*np.sin(np.pi/6)
Rf   = (-f1-f2-f3)*l

print(Rf)

np_t = np.array(t).reshape((-1, 1))
np_posex = np.array(x).reshape((-1, 1))
np_posey = np.array(y).reshape((-1, 1))
np_poseTheta = np.array(z).reshape((-1, 1))
np_posx = np.array(rs1).reshape((-1, 1))
np_posy = np.array(rs2).reshape((-1, 1))
np_rpm1 = np.array(rpm1).reshape((-1, 1))
np_rpm2 = np.array(rpm2).reshape((-1, 1))
np_rpm3 = np.array(rpm3).reshape((-1, 1))
np_m1_cur = np.array(m1_cur).reshape((-1, 1))
np_m2_cur = np.array(m2_cur).reshape((-1, 1))
np_m3_cur = np.array(m3_cur).reshape((-1, 1))
np_vx = np.array(vx).reshape((-1, 1))
np_vy = np.array(vy).reshape((-1, 1))
np_vw = np.array(vw).reshape((-1, 1))
np_v1 = np.array(v1).reshape((-1, 1))
np_v2 = np.array(v2).reshape((-1, 1))
np_v3 = np.array(v3).reshape((-1, 1))
np_outputAll = np.hstack((np_posex,np_posey,np_poseTheta,np_posx,np_posy,np_rpm1,np_rpm2,np_rpm3,np_v1,np_v2,np_v3,np_m1_cur,np_m2_cur,np_m3_cur,np_vx,np_vy,np_vw,np_t))
# ~ np.savetxt(my_txt1+'_fixed.txt', np_outputAll, delimiter=',',fmt='%2.4f')

#####################################################		fv
Fv1 = 264.542360034
Fv2 = 105.179492544
Fv3 = -6.37451469962
Fv4 = -37.7158786394

v1 = 0.3
v2 = 0.5
v3 = 0.7
v4 = 0.76

v_p = np.arange(0.3,0.76,0.01)
Fv_p = -651.5344 *v_p + 449.5248

#####################################################		fvn
Fvn1 = 407.136260417
Fvn2 = 513.405725
Fvn3 = 595.178007812
Fvn4 = 660.733846354

vn1 = 0.3
vn2 = 0.4
vn3 = 0.5
vn4 = 0.6

vn_p = np.arange(0.3,0.6004,0.01)
Fvn_p = 842.5650*vn_p + 164.9592

#####################################################		R
Rf1 = -22.8134318125
Rf2 = -51.920224125
Rf3 = -130.587230375
Rf4 = -231.674333406

w1 = 0.5
w2 = 1
w3 = 2
w4 = 3

w = np.arange(0.5,3,0.01)
Rf_p =  -83.9070*w + 27.1001


fig = plt.figure()
txt = 'Fv_Fvn_Rf'
# ~ fig.suptitle('Fv = 266.6164*v + 59.6423 & Fvn = 219.6386*vn + 160.7826 & Rf =  85.0661*w -2.8055',fontsize=16)

ax1 = fig.add_subplot(131)
ax2 = fig.add_subplot(132)
ax3 = fig.add_subplot(133)

ax1 = plt.subplot(131)
ax1.plot(v1,Fv1,'r*',linewidth=3)
ax1.plot(v2,Fv2,'r*',linewidth=3)
ax1.plot(v3,Fv3,'r*',linewidth=3)
ax1.plot(v4,Fv4,'r*',linewidth=3)
ax1.plot(v_p,Fv_p,'b',linewidth=3)
# ~ ax1.text(0.3,260,'Bv = 266.6164',color='green', fontsize=9)
# ~ ax1.text(0.3,255,'Cv = 59.6423',color='green', fontsize=9)
plt.xlabel('v(m/s)')
plt.ylabel('Fv(N)')
plt.grid(True)

ax2 = plt.subplot(132)
ax2.plot(vn1,Fvn1,'r*',linewidth=3)
ax2.plot(vn2,Fvn2,'r*',linewidth=3)
ax2.plot(vn3,Fvn3,'r*',linewidth=3)
ax2.plot(vn4,Fvn4,'r*',linewidth=3)
ax2.plot(vn_p,Fvn_p,'b',linewidth=3)
ax2.text(0.3,290,'Bvn = 219.6386',color='green', fontsize=9)
ax2.text(0.3,285,'Cvn = 160.7826',color='green', fontsize=9)
plt.xlabel('v(m/s)')
plt.ylabel('Fvn(N)')
plt.grid(True)

ax3 = plt.subplot(133)
ax3.plot(w1,Rf1,'r*',linewidth=3)
ax3.plot(w2,Rf2,'r*',linewidth=3)
ax3.plot(w3,Rf3,'r*',linewidth=3)
ax3.plot(w4,Rf4,'r*',linewidth=3)
ax3.plot(w,Rf_p,'b',linewidth=3)
ax3.text(0.5,245,'Bw = 85.0661',color='green', fontsize=9)
ax3.text(0.5,235,'Cw = -2.8055',color='green', fontsize=9)
plt.xlabel('w(rad/s)')
plt.ylabel('Rf(N)')
plt.grid(True)


#Save image into full size 
fig = plt.gcf()
fig.set_size_inches((15, 15), forward=False)
fig.savefig(txt+'.png', dpi=200)

#will not stop the figure if use this
plt.show()
plt.draw()

#the figure will be closed after 3s
plt.show(block=False)
plt.draw()
# ~ time.sleep(3)

# ~ plt.close()
