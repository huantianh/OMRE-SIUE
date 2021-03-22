import numpy as np
import matplotlib.pyplot as plt
import sympy as sym
from sympy import *
import numpy as np

# ~ R = 0.2
# ~ speed = 0.5
# ~ t = 1
# ~ xc = 0
# ~ yc = 0
# ~ thetac = 0


k = Kx = Ky = Kz = 5

t , R, speed, xc, yc, thetac, m1_rpm, c_rpm1, m2_rpm, c_rpm2, m3_rpm, c_rpm3 = sym.symbols('t R speed xc yc thetac m1_rpm c_rpm1 m2_rpm c_rpm2 m3_rpm c_rpm3')

r = 0.03
l = 0.19

j = (2*pi*r/60)*Matrix([
[(2/3)*sin(thetac+pi/3),(-2/3)*sin(thetac),(2/3)*sin(thetac-pi/3)],
[(-2/3)*cos(thetac+pi/3),(2/3)*cos(thetac),(-2/3)*cos(thetac-pi/3)],
[-1/(3*l),-1/(3*l),-1/(3*l)]
])

j_dot = np.array([(r*np.pi*np.cos(thetac+np.pi/3))/45, -(r*np.pi*np.cos(thetac))/45, (r*np.pi*np.cos(thetac-np.pi/3))/45, (r*np.pi*np.sin(thetac+np.pi/3))/45, -(r*np.pi*np.sin(thetac))/45, (r*np.pi*np.sin(thetac-np.pi/3))/45, 0, 0, 0]).reshape(3,3)


# ~ j_dot = sym.diff(j,thetac)
j_ddot = sym.diff(j_dot,thetac)
print(j_dot)

j_inv = j.inv()
j_inv_dot = sym.diff(j_inv,thetac)
print(j_inv_dot)



j_trans = j.transpose()
j_trans_dot = sym.diff(j_trans,thetac)

xd = R*sin(speed*t)
yd = R*cos(speed*t)
thetad = 0

xd_dot = sym.diff(xd,t)
yd_dot = sym.diff(yd,t)
thetad_dot = sym.diff(thetad,t)

qd_dot = Matrix([
[xd_dot],
[yd_dot],
[thetad_dot]
])

K = Matrix([
[1,0,0],
[0,1,0],
[0,0,1]
])

e1 = Matrix([
[xc-xd],
[yc-yd],
[thetac-thetad]
])

w1 = -K*e1+qd_dot
wd = j_inv*w1
# ~ wd_dot = sym.diff(wd,thetac,t)
wd_dot = sym.diff(wd,t)

e2 = Matrix([
[m2_rpm-c_rpm2],
[m1_rpm-c_rpm1],
[m3_rpm-c_rpm3]
])

a1 = 12.17; b1 = 224.46; a2 = 4.74; b2 = 10.08; c2 = 0.32;

z1 = j*e2
j1 = j_inv*j_dot
j2 = j_inv*e1

vd = 1/b1*a1*wd + 1/b1*wd_dot - 1/b1*j1*e2 - 1/b1*j2
vd_dot = sym.diff(vd, thetac,t)
# ~ print(wd_dot)


j3 = j_trans*z1
u1 = (1/b2*(vd_dot + a2*vd + b1*j3))
c3 = 1/c2

u11 = (u1.row(0))**c3
u21 = (u1.row(1))**c3
u31 = (u1.row(2))**c3

u_equ = Matrix([
[u11],
[u21],
[u31]
])

u_num = (t, R, speed, xc, yc, thetac, m1_rpm, c_rpm1, m2_rpm, c_rpm2, m3_rpm, c_rpm3)
u_final = lambdify(u_num, u_equ, modules='numpy')
u2 = u_final(30,0.2,0.3,1,1,1,1,1,1,1,1,1)

# ~ print (vd_dot[2])



# ~ A = sqrt(3)*R*speed*(333.333333333333*(speed*sin(speed*t) - cos(speed*t))*(sin(thetac) - cos(thetac + pi/6)) - 333.333333333333*(speed*cos(speed*t) + sin(speed*t))*((-sin(thetac) + cos(thetac + pi/6))*(sin(thetac) + sin(thetac + pi/3)) + (sin(thetac + pi/6) + cos(thetac))*(cos(thetac) + cos(thetac + pi/3)))/(cos(thetac) + cos(thetac + pi/3)) - 7.91700075e+19*(speed*cos(speed*t) + sin(speed*t))*(4.2103486390769e-18*(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) + cos(thetac)) - 6.31552295861536e-18*sqrt(3))*(sin(thetac) + sin(thetac + pi/3))/(cos(thetac) + cos(thetac + pi/3))**2)/pi
# ~ B = sqrt(3)*R*speed*(333.333333333333*(speed*sin(speed*t) - cos(speed*t))*(sin(thetac + pi/3) + cos(thetac + pi/6)) + 1.3889475e+17*(speed*cos(speed*t) + sin(speed*t))*(2.39989872427384e-15*(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) - cos(thetac + pi/3)) - 3.59984808641075e-15*sqrt(3))*(sin(thetac) + sin(thetac + pi/3))/(cos(thetac) + cos(thetac + pi/3))**2 + 333.333333333333*(speed*cos(speed*t) + sin(speed*t))*((sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/3) + cos(thetac + pi/6)) + (sin(thetac + pi/6) - cos(thetac + pi/3))*(cos(thetac) + cos(thetac + pi/3)))/(cos(thetac) + cos(thetac + pi/3)))/pi
# ~ C = 333.333333333333*sqrt(3)*R*speed*(-(speed*sin(speed*t) - cos(speed*t))*(sin(thetac) + sin(thetac + pi/3)) + (speed*cos(speed*t) + sin(speed*t))*(cos(thetac) + cos(thetac + pi/3)))/pi

# ~ A1= sqrt(3)*R*speed*(-0.00445513677269892*speed*(666.666666666667*(-speed*sin(speed*t) + cos(speed*t))*((-sin(thetac) + cos(thetac + pi/6))*(sin(thetac) + sin(thetac + pi/3)) + (sin(thetac + pi/6) + cos(thetac))*(cos(thetac) + cos(thetac + pi/3)))*(sin(thetac) + sin(thetac + pi/3))/(cos(thetac) + cos(thetac + pi/3))**2 + 666.666666666667*(-speed*sin(speed*t) + cos(speed*t))*((-sin(thetac) + cos(thetac + pi/6))*(cos(thetac) + cos(thetac + pi/3)) - (sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) + cos(thetac)))/(cos(thetac) + cos(thetac + pi/3)) + 1.58340015e+20*(-speed*sin(speed*t) + cos(speed*t))*(4.2103486390769e-18*(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) + cos(thetac)) - 6.31552295861536e-18*sqrt(3))*(sin(thetac) + sin(thetac + pi/3))**2/(cos(thetac) + cos(thetac + pi/3))**3 + 7.91700075e+19*(-speed*sin(speed*t) + cos(speed*t))*(4.2103486390769e-18*(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) + cos(thetac)) - 6.31552295861536e-18*sqrt(3))/(cos(thetac) + cos(thetac + pi/3)) - 333.333333333333*(speed*cos(speed*t) + sin(speed*t))*(sin(thetac + pi/6) + cos(thetac))) + 18.0730048412486*(speed*sin(speed*t) - cos(speed*t))*(sin(thetac) - cos(thetac + pi/6)) - 18.0730048412486*(speed*cos(speed*t) + sin(speed*t))*((-sin(thetac) + cos(thetac + pi/6))*(sin(thetac) + sin(thetac + pi/3)) + (sin(thetac + pi/6) + cos(thetac))*(cos(thetac) + cos(thetac + pi/3)))/(cos(thetac) + cos(thetac + pi/3)) - 4.29251978648757e+18*(speed*cos(speed*t) + sin(speed*t))*(4.2103486390769e-18*(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) + cos(thetac)) - 6.31552295861536e-18*sqrt(3))*(sin(thetac) + sin(thetac + pi/3))/(cos(thetac) + cos(thetac + pi/3))**2 - 1.48504559089964*((-sin(thetac) + cos(thetac + pi/6))*(sin(thetac) + sin(thetac + pi/3)) + (sin(thetac + pi/6) + cos(thetac))*(cos(thetac) + cos(thetac + pi/3)))*sin(speed*t)/(cos(thetac) + cos(thetac + pi/3)) - 3.52713211708099e+17*(4.2103486390769e-18*(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) + cos(thetac)) - 6.31552295861536e-18*sqrt(3))*(sin(thetac) + sin(thetac + pi/3))*sin(speed*t)/(cos(thetac) + cos(thetac + pi/3))**2 - 1.48504559089964*(sin(thetac) - cos(thetac + pi/6))*cos(speed*t))/pi
# ~ B1= sqrt(3)*R*speed*(-0.00445513677269892*speed*(666.666666666667*(speed*sin(speed*t) - cos(speed*t))*(-(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) - cos(thetac + pi/3)) + (sin(thetac + pi/3) + cos(thetac + pi/6))*(cos(thetac) + cos(thetac + pi/3)))/(cos(thetac) + cos(thetac + pi/3)) + 2.777895e+17*(speed*sin(speed*t) - cos(speed*t))*(2.39989872427384e-15*(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) - cos(thetac + pi/3)) - 3.59984808641075e-15*sqrt(3))*(sin(thetac) + sin(thetac + pi/3))**2/(cos(thetac) + cos(thetac + pi/3))**3 + 1.3889475e+17*(speed*sin(speed*t) - cos(speed*t))*(2.39989872427384e-15*(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) - cos(thetac + pi/3)) - 3.59984808641075e-15*sqrt(3))/(cos(thetac) + cos(thetac + pi/3)) + 666.666666666667*(speed*sin(speed*t) - cos(speed*t))*((sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/3) + cos(thetac + pi/6)) + (sin(thetac + pi/6) - cos(thetac + pi/3))*(cos(thetac) + cos(thetac + pi/3)))*(sin(thetac) + sin(thetac + pi/3))/(cos(thetac) + cos(thetac + pi/3))**2 + 333.333333333333*(speed*cos(speed*t) + sin(speed*t))*(sin(thetac + pi/6) - cos(thetac + pi/3))) + 18.0730048412486*(speed*sin(speed*t) - cos(speed*t))*(sin(thetac + pi/3) + cos(thetac + pi/6)) + 7.53073646752205e+15*(speed*cos(speed*t) + sin(speed*t))*(2.39989872427384e-15*(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) - cos(thetac + pi/3)) - 3.59984808641075e-15*sqrt(3))*(sin(thetac) + sin(thetac + pi/3))/(cos(thetac) + cos(thetac + pi/3))**2 + 18.0730048412486*(speed*cos(speed*t) + sin(speed*t))*((sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/3) + cos(thetac + pi/6)) + (sin(thetac + pi/6) - cos(thetac + pi/3))*(cos(thetac) + cos(thetac + pi/3)))/(cos(thetac) + cos(thetac + pi/3)) + 618795108259824.0*(2.39989872427384e-15*(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) - cos(thetac + pi/3)) - 3.59984808641075e-15*sqrt(3))*(sin(thetac) + sin(thetac + pi/3))*sin(speed*t)/(cos(thetac) + cos(thetac + pi/3))**2 + 1.48504559089964*((sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/3) + cos(thetac + pi/6)) + (sin(thetac + pi/6) - cos(thetac + pi/3))*(cos(thetac) + cos(thetac + pi/3)))*sin(speed*t)/(cos(thetac) + cos(thetac + pi/3)) - 1.48504559089964*(sin(thetac + pi/3) + cos(thetac + pi/6))*cos(speed*t))/pi
# ~ C1= sqrt(3)*R*speed*(1.48504559089964*speed*((speed*sin(speed*t) - cos(speed*t))*(sin(thetac) + sin(thetac + pi/3)) - (speed*cos(speed*t) + sin(speed*t))*(cos(thetac) + cos(thetac + pi/3))) - 18.0730048412486*(speed*sin(speed*t) - cos(speed*t))*(sin(thetac) + sin(thetac + pi/3)) + 18.0730048412486*(speed*cos(speed*t) + sin(speed*t))*(cos(thetac) + cos(thetac + pi/3)) + 1.48504559089964*(sin(thetac) + sin(thetac + pi/3))*cos(speed*t) + 1.48504559089964*(cos(thetac) + cos(thetac + pi/3))*sin(speed*t))/pi

##################numpy
# ~ A = np.sqrt(3)*R*speed*(333.333333333333*(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac) - np.cos(thetac + np.pi/6)) - 333.333333333333*(speed*np.cos(speed*t) + np.sin(speed*t))*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + (np.sin(thetac + np.pi/6) + np.cos(thetac))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) - 7.91700075e+19*(speed*np.cos(speed*t) + np.sin(speed*t))*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2)/np.pi
# ~ B = np.sqrt(3)*R*speed*(333.333333333333*(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6)) + 1.3889475e+17*(speed*np.cos(speed*t) + np.sin(speed*t))*(2.39989872427384e-15*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3)) - 3.59984808641075e-15*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 + 333.333333333333*(speed*np.cos(speed*t) + np.sin(speed*t))*((np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6)) + (np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)))/np.pi
# ~ C = 333.333333333333*np.sqrt(3)*R*speed*(-(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + (speed*np.cos(speed*t) + np.sin(speed*t))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/np.pi
# ~ wd_dot = np.array([A,B,C]).reshape(3,1)


# ~ A1= np.sqrt(3)*R*speed*(-0.00445513677269892*speed*(666.666666666667*(-speed*np.sin(speed*t) + np.cos(speed*t))*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + (np.sin(thetac + np.pi/6) + np.cos(thetac))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 + 666.666666666667*(-speed*np.sin(speed*t) + np.cos(speed*t))*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.cos(thetac) + np.cos(thetac + np.pi/3)) - (np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) + 1.58340015e+20*(-speed*np.sin(speed*t) + np.cos(speed*t))*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))**2/(np.cos(thetac) + np.cos(thetac + np.pi/3))**3 + 7.91700075e+19*(-speed*np.sin(speed*t) + np.cos(speed*t))*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) - 333.333333333333*(speed*np.cos(speed*t) + np.sin(speed*t))*(np.sin(thetac + np.pi/6) + np.cos(thetac))) + 18.0730048412486*(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac) - np.cos(thetac + np.pi/6)) - 18.0730048412486*(speed*np.cos(speed*t) + np.sin(speed*t))*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.sin(thetac) + sin(thetac + np.pi/3)) + (np.sin(thetac + np.pi/6) + np.cos(thetac))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) - 4.29251978648757e+18*(speed*np.cos(speed*t) + np.sin(speed*t))*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 - 1.48504559089964*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + (np.sin(thetac + np.pi/6) + np.cos(thetac))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))*np.sin(speed*t)/(np.cos(thetac) + np.cos(thetac + np.pi/3)) - 3.52713211708099e+17*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))*np.sin(speed*t)/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 - 1.48504559089964*(np.sin(thetac) - np.cos(thetac + np.pi/6))*np.cos(speed*t))/np.pi
# ~ B1= np.sqrt(3)*R*speed*(-0.00445513677269892*speed*(666.666666666667*(speed*sin(speed*t) - cos(speed*t))*(-(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) - cos(thetac + pi/3)) + (sin(thetac + pi/3) + cos(thetac + pi/6))*(cos(thetac) + cos(thetac + pi/3)))/(cos(thetac) + cos(thetac + pi/3)) + 2.777895e+17*(speed*sin(speed*t) - cos(speed*t))*(2.39989872427384e-15*(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) - cos(thetac + pi/3)) - 3.59984808641075e-15*np.sqrt(3))*(sin(thetac) + sin(thetac + pi/3))**2/(cos(thetac) + cos(thetac + pi/3))**3 + 1.3889475e+17*(speed*sin(speed*t) - cos(speed*t))*(2.39989872427384e-15*(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) - cos(thetac + pi/3)) - 3.59984808641075e-15*np.sqrt(3))/(cos(thetac) + cos(thetac + pi/3)) + 666.666666666667*(speed*sin(speed*t) - cos(speed*t))*((sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/3) + cos(thetac + pi/6)) + (sin(thetac + pi/6) - cos(thetac + pi/3))*(cos(thetac) + cos(thetac + pi/3)))*(sin(thetac) + sin(thetac + pi/3))/(cos(thetac) + cos(thetac + pi/3))**2 + 333.333333333333*(speed*cos(speed*t) + sin(speed*t))*(sin(thetac + pi/6) - cos(thetac + pi/3))) + 18.0730048412486*(speed*sin(speed*t) - cos(speed*t))*(sin(thetac + pi/3) + cos(thetac + pi/6)) + 7.53073646752205e+15*(speed*cos(speed*t) + sin(speed*t))*(2.39989872427384e-15*(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) - cos(thetac + pi/3)) - 3.59984808641075e-15*np.sqrt(3))*(sin(thetac) + sin(thetac + pi/3))/(cos(thetac) + cos(thetac + pi/3))**2 + 18.0730048412486*(speed*cos(speed*t) + sin(speed*t))*((sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/3) + cos(thetac + pi/6)) + (sin(thetac + pi/6) - cos(thetac + pi/3))*(cos(thetac) + cos(thetac + pi/3)))/(cos(thetac) + cos(thetac + pi/3)) + 618795108259824.0*(2.39989872427384e-15*(sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/6) - cos(thetac + pi/3)) - 3.59984808641075e-15*sqrt(3))*(sin(thetac) + sin(thetac + pi/3))*sin(speed*t)/(cos(thetac) + cos(thetac + pi/3))**2 + 1.48504559089964*((sin(thetac) + sin(thetac + pi/3))*(sin(thetac + pi/3) + cos(thetac + pi/6)) + (sin(thetac + pi/6) - cos(thetac + pi/3))*(cos(thetac) + cos(thetac + pi/3)))*sin(speed*t)/(cos(thetac) + cos(thetac + pi/3)) - 1.48504559089964*(sin(thetac + pi/3) + cos(thetac + pi/6))*cos(speed*t))/pi
# ~ C1= np.sqrt(3)*R*speed*(1.48504559089964*speed*((speed*sin(speed*t) - cos(speed*t))*(sin(thetac) + sin(thetac + pi/3)) - (speed*cos(speed*t) + sin(speed*t))*(cos(thetac) + cos(thetac + pi/3))) - 18.0730048412486*(speed*sin(speed*t) - cos(speed*t))*(sin(thetac) + sin(thetac + pi/3)) + 18.0730048412486*(speed*cos(speed*t) + sin(speed*t))*(cos(thetac) + cos(thetac + pi/3)) + 1.48504559089964*(sin(thetac) + sin(thetac + pi/3))*cos(speed*t) + 1.48504559089964*(cos(thetac) + cos(thetac + pi/3))*sin(speed*t))/pi
# ~ vd_dot = np.array([A1,B1,C1]).reshape(3,1)



















































# ~ thetad = sym.Symbol('thetad')

# ~ thetad = 0

# ~ j = (2*np.pi*0.03/60)*np.array([(2/3)*np.sin(thetad+np.pi/3),(-2/3)*np.sin(thetad),(2/3)*np.sin(thetad-np.pi/3),(-2/3)*np.cos(thetad+np.pi/3),(2/3)*np.cos(thetad),(-2/3)*np.cos(thetad-np.pi/3),-1/(3*0.19),-1/(3*0.19),-1/(3*0.19)]).reshape(3,3)
# ~ j_inv = np.linalg.inv(j).reshape(3,3)

# ~ j_inv_dot = sym.diff(j)
# ~ print(j)


# ~ R = 0.5
# ~ a = 0
# ~ b = 0
# ~ step = 0.01

# ~ y1 = np.arange(R,-R,-step)
# ~ w1 = R*R - (y1-b)*(y1-b)
# ~ x1 = a + np.sqrt(w1)

# ~ y2 = np.arange(-R,R,step)
# ~ y3 = np.append(y2,0)
# ~ w2 = R*R - (y2-b)*(y2-b)
# ~ x2 = a - np.sqrt(w2)

# ~ print(y2)



# ~ t = np.arange(0,2.1,0.1)
						
# ~ for i in t:
	# ~ xd = np.sin(0.1*i)
	# ~ yd = np.cos(0.1*i)
	# ~ thetad = 0
	# ~ print(str(i)+' , '+str(xd)+' , '+str(yd))



		
# ~ # First create some toy data:
# ~ x = np.linspace(0, 2*np.pi, 400)
# ~ y = np.sin(x**2)

# ~ # Creates just a figure and only one subplot
# ~ fig, ax = plt.subplots()
# ~ ax.plot(x, y)
# ~ ax.set_title('Simple plot')

# ~ # Creates two subplots and unpacks the output array immediately
# ~ f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
# ~ ax1.plot(x, y)
# ~ ax1.set_title('Sharing Y axis')
# ~ ax2.scatter(x, y)

# ~ # Creates four polar axes, and accesses them through the returned array
# ~ fig, axes = plt.subplots(2, 2, subplot_kw=dict(polar=True))
# ~ axes[0, 0].plot(x, y)
# ~ axes[1, 1].scatter(x, y)

# ~ # Share a X axis with each column of subplots
# ~ plt.subplots(2, 2, sharex='col')

# ~ # Share a Y axis with each row of subplots
# ~ plt.subplots(2, 2, sharey='row')

# ~ # Share both X and Y axes with all subplots
# ~ plt.subplots(2, 2, sharex='all', sharey='all')

# ~ # Note that this is the same as
# ~ plt.subplots(2, 2, sharex=True, sharey=True)

# ~ # Creates figure number 10 with a single subplot
# ~ # and clears it if it already exists.
# ~ fig, ax=plt.subplots(num=10, clear=True)

# ~ plt.show()
