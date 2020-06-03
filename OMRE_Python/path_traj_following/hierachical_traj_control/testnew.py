import numpy as np
import matplotlib.pyplot as plt
import sympy as sym
from sympy import *

R = 0.2
speed = 0.5
t = 1
xc = 0
yc = 0
thetac = 0

A = np.sqrt(3)*R*speed*(333.333333333333*(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac) - np.cos(thetac + np.pi/6)) - 333.333333333333*(speed*np.cos(speed*t) + np.sin(speed*t))*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + (np.sin(thetac + np.pi/6) + np.cos(thetac))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) - 7.91700075e+19*(speed*np.cos(speed*t) + np.sin(speed*t))*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2)/np.pi
B = np.sqrt(3)*R*speed*(333.333333333333*(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6)) + 1.3889475e+17*(speed*np.cos(speed*t) + np.sin(speed*t))*(2.39989872427384e-15*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3)) - 3.59984808641075e-15*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 + 333.333333333333*(speed*np.cos(speed*t) + np.sin(speed*t))*((np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6)) + (np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)))/np.pi
C = 333.333333333333*np.sqrt(3)*R*speed*(-(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + (speed*np.cos(speed*t) + np.sin(speed*t))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/np.pi
wd_dot = np.array([A,B,C]).reshape(3,1)

A1= np.sqrt(3)*R*speed*(-0.00445513677269892*speed*(666.666666666667*(-speed*np.sin(speed*t) + np.cos(speed*t))*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + (np.sin(thetac + np.pi/6) + np.cos(thetac))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 + 666.666666666667*(-speed*np.sin(speed*t) + np.cos(speed*t))*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.cos(thetac) + np.cos(thetac + np.pi/3)) - (np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) + 1.58340015e+20*(-speed*np.sin(speed*t) + np.cos(speed*t))*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))**2/(np.cos(thetac) + np.cos(thetac + np.pi/3))**3 + 7.91700075e+19*(-speed*np.sin(speed*t) + np.cos(speed*t))*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) - 333.333333333333*(speed*np.cos(speed*t) + np.sin(speed*t))*(np.sin(thetac + np.pi/6) + np.cos(thetac))) + 18.0730048412486*(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac) - np.cos(thetac + np.pi/6)) - 18.0730048412486*(speed*np.cos(speed*t) + np.sin(speed*t))*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + (np.sin(thetac + np.pi/6) + np.cos(thetac))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) - 4.29251978648757e+18*(speed*np.cos(speed*t) + np.sin(speed*t))*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 - 1.48504559089964*((-np.sin(thetac) + np.cos(thetac + np.pi/6))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + (np.sin(thetac + np.pi/6) + np.cos(thetac))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))*np.sin(speed*t)/(np.cos(thetac) + np.cos(thetac + np.pi/3)) - 3.52713211708099e+17*(4.2103486390769e-18*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) + np.cos(thetac)) - 6.31552295861536e-18*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))*np.sin(speed*t)/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 - 1.48504559089964*(np.sin(thetac) - np.cos(thetac + np.pi/6))*np.cos(speed*t))/np.pi
B1= np.sqrt(3)*R*speed*(-0.00445513677269892*speed*(666.666666666667*(speed*np.sin(speed*t) - np.cos(speed*t))*(-(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3)) + (np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) + 2.777895e+17*(speed*np.sin(speed*t) - np.cos(speed*t))*(2.39989872427384e-15*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3)) - 3.59984808641075e-15*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))**2/(np.cos(thetac) + np.cos(thetac + np.pi/3))**3 + 1.3889475e+17*(speed*np.sin(speed*t) - np.cos(speed*t))*(2.39989872427384e-15*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3)) - 3.59984808641075e-15*np.sqrt(3))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) + 666.666666666667*(speed*np.sin(speed*t) - np.cos(speed*t))*((np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6)) + (np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 + 333.333333333333*(speed*np.cos(speed*t) + np.sin(speed*t))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3))) + 18.0730048412486*(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6)) + 7.53073646752205e+15*(speed*np.cos(speed*t) + np.sin(speed*t))*(2.39989872427384e-15*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3)) - 3.59984808641075e-15*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 + 18.0730048412486*(speed*np.cos(speed*t) + np.sin(speed*t))*((np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6)) + (np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))/(np.cos(thetac) + np.cos(thetac + np.pi/3)) + 618795108259824.0*(2.39989872427384e-15*(np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3)) - 3.59984808641075e-15*np.sqrt(3))*(np.sin(thetac) + np.sin(thetac + np.pi/3))*np.sin(speed*t)/(np.cos(thetac) + np.cos(thetac + np.pi/3))**2 + 1.48504559089964*((np.sin(thetac) + np.sin(thetac + np.pi/3))*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6)) + (np.sin(thetac + np.pi/6) - np.cos(thetac + np.pi/3))*(np.cos(thetac) + np.cos(thetac + np.pi/3)))*np.sin(speed*t)/(np.cos(thetac) + np.cos(thetac + np.pi/3)) - 1.48504559089964*(np.sin(thetac + np.pi/3) + np.cos(thetac + np.pi/6))*np.cos(speed*t))/np.pi
C1= np.sqrt(3)*R*speed*(1.48504559089964*speed*((speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) - (speed*np.cos(speed*t) + np.sin(speed*t))*(np.cos(thetac) + np.cos(thetac + np.pi/3))) - 18.0730048412486*(speed*np.sin(speed*t) - np.cos(speed*t))*(np.sin(thetac) + np.sin(thetac + np.pi/3)) + 18.0730048412486*(speed*np.cos(speed*t) + np.sin(speed*t))*(np.cos(thetac) + np.cos(thetac + np.pi/3)) + 1.48504559089964*(np.sin(thetac) + np.sin(thetac + np.pi/3))*np.cos(speed*t) + 1.48504559089964*(np.cos(thetac) + np.cos(thetac + np.pi/3))*np.sin(speed*t))/np.pi
vd_dot = np.array([A1,B1,C1]).reshape(3,1)


print(wd_dot)












































