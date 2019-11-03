import numpy as np
import matplotlib.pyplot as plt
import sympy as sym

thetad = sym.Symbol('thetad')

# ~ thetad = 0

j = (2*np.pi*0.03/60)*np.array([(2/3)*np.sin(thetad+np.pi/3),(-2/3)*np.sin(thetad),(2/3)*np.sin(thetad-np.pi/3),(-2/3)*np.cos(thetad+np.pi/3),(2/3)*np.cos(thetad),(-2/3)*np.cos(thetad-np.pi/3),-1/(3*0.19),-1/(3*0.19),-1/(3*0.19)]).reshape(3,3)
j_inv = np.linalg.inv(j).reshape(3,3)

j_inv_dot = sym.diff(j)
print(j)















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
