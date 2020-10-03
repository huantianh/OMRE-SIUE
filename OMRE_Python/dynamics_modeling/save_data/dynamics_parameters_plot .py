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
vw = []
Fv= []
Fvn = []
R_f = []
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
Kt1 = 1.16516 / 5.6			#Motor Torque Constant
Kt2 = 1.16516 / 5.6			#Motor Torque Constant
Kt3 = 1.16516 / 5.6			#Motor Torque Constant
r = 0.03					#Wheel Radius
l = 0.19                    #distance from wheel to CG

T1 = li*Kt1*i1
T2 = li*Kt2*i2
T3 = li*Kt3*i3

f1 = T1/r
f2 = T2/r
f3 = T3/r

Fv = f2*np.cos(30) - f3*np.cos(30)
Fvn = f1 - f2*np.sin(30) - f3*np.sin(30)
R_f = (-f1-f2-f3)*l

print(R_f)

#####################################################		Fv
Fv1 = 134.521450247
Fv2 = 201.561397973
Fv3 = 250.843313995
Fv4 = 255.161385259

v1 = 0.30109
v2 = 0.50061
v3 = 0.70013
v4 = 0.76179

v_p = np.arange(0.3,0.76179,0.01)
Fv_p = 266.6164 *v_p + 59.6423

#####################################################		Fvn
Fvn1 = 216.321479776
Fvn2 = 257.508844057
Fvn3 = 286.898612441
Fvn4 = 279.156706357

vn1 = 0.30159
vn2 = 0.40212
vn3 = 0.50265
vn4 = 0.60004

vn_p = np.arange(0.3,0.6004,0.01)
Fvn_p = 219.6386*vn_p + 160.7826

#####################################################		R_f
Rf1 = 42.2331330322
Rf2 = 79.0366181184
Rf3 = 167.510730087
Rf4 = 252.927091313

w1 = 0.5
w2 = 1
w3 = 2
w4 = 3

w = np.arange(0.5,3,0.01)
Rf_p =  85.0661*w -2.8055


fig = plt.figure()
txt = 'Fv_Fvn_Rf'
fig.suptitle('Fv = 266.6164*v + 59.6423 & Fvn = 219.6386*vn + 160.7826 & Rf =  85.0661*w -2.8055',fontsize=16)

ax1 = fig.add_subplot(131)
ax2 = fig.add_subplot(132)
ax2 = fig.add_subplot(133)

ax1 = plt.subplot(131)
ax1.plot(v1,Fv1,'r*',linewidth=3)
ax1.plot(v2,Fv2,'r*',linewidth=3)
ax1.plot(v3,Fv3,'r*',linewidth=3)
ax1.plot(v4,Fv4,'r*',linewidth=3)
ax1.plot(v_p,Fv_p,'b',linewidth=3)
ax1.text(0.3,260,'Bv = 266.6164',color='green', fontsize=9)
ax1.text(0.3,255,'Cv = 59.6423',color='green', fontsize=9)
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
# ~ plt.draw()
# ~ time.sleep(3)

plt.close()
