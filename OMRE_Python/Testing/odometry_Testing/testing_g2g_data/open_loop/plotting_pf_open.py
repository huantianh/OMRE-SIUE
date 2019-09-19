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

with open(root.filename,'r') as csvfile:
	plots = csv.reader(csvfile, delimiter=',')
	for idx, row in enumerate(plots):
		x.append(float(row[0]))
		y.append(float(row[1]))
		z.append(float(row[2]))
		
		
		
fig, ax = plt.subplots()
ax.plot(x,y, 'b', label='Circle', linewidth=3)

plt.legend()

#square
# ~ ax.vlines(x=0, ymin=0, ymax=1,color='r',linewidth=1.5)
# ~ ax.vlines(x=1, ymin=0, ymax=1,color='r',linewidth=1.5)
# ~ ax.hlines(y=0, xmin=0, xmax=1,color='r',linewidth=1.5)
# ~ ax.hlines(y=1, xmin=0, xmax=1,color='r',linewidth=1.5)
# ~ plt.axis([0,2.1,0,2.1])

#triangle
# ~ plt.plot([0,1,2],[0,1,0],'r',linewidth=1.5)
# ~ ax.hlines(y=0, xmin=0, xmax=2,color='r',linewidth=1.5)
# ~ plt.axis([-0.2,2.2,-0.2,2.2])

#~ plt.autoscale(enable=True, axis='x', tight=True)
#~ plt.autoscale(enable=True, axis='y', tight=False)
plt.xlabel('X_open')
plt.ylabel('Y_open')
plt.title(my_txt1)

#Save image into full size 
fig = plt.gcf()
fig.set_size_inches((20, 11), forward=False)
fig.savefig(my_txt1+'.png', dpi=100)

#will not stop the figure if use this
plt.show()
plt.draw()

#the figure will be closed after 3s
#plt.show(block=False)
#plt.draw()
#time.sleep(3)

plt.close()
