import numpy as np
import matplotlib.pyplot as plt


b = 0.5
a = 0
R = 0.5

y1 = np.arange(0,b*2,0.01)
w1 = R*R - (y1-b)*(y1-b)
x1 = a + np.sqrt(w1)

y2 = np.arange(b*2,0,-0.01)
y3 = np.append(y2,0)
w2 = R*R - (y3-b)*(y3-b)
x2 = a - np.sqrt(w2)

x = np.concatenate((x1,x2))
y = np.concatenate((y1,y3))

for i,j in zip(x,y):
		xd = i
		yd = j
		thetad = 0
		print(str(i) +' , '+str(j))


# ~ Kx = 2
# ~ Ky = 2
# ~ Kz = 2

# ~ K = np.array([Kx,0,0,0,Ky,0,0,0,Kz]).reshape(3,3)





		
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
