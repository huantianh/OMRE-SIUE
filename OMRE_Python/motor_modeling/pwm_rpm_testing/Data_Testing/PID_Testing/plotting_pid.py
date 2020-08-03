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
y1 = []
y2 = []
y3 = []

with open(root.filename,'r') as csvfile:
	plots = csv.reader(csvfile, delimiter=',')
	for idx, row in enumerate(plots):
		if idx == 0:
			x0 = float(row[0])
		x.append(float(row[0])-x0)
		y1.append(float(row[1]))
		y2.append(float(row[2]))
		y3.append(float(row[3]))

fig = plt.plot(x,y1, 'b', label='Motor1', linewidth=2)
fig = plt.plot(x,y2, 'g', label='Motor2', linewidth=2)
fig = plt.plot(x,y3, 'r', label='Motor3', linewidth=2)
plt.legend()
plt.plot(x,150*np.ones(len(x)),'m')
plt.autoscale(enable=True, axis='x', tight=True)
plt.xlabel('Time(seconds)')
plt.ylabel('RPM')
plt.title(my_txt1)

#Save image into full size 
fig = plt.gcf()
fig.set_size_inches((20, 11), forward=False)
fig.savefig(my_txt1+'.png', dpi=500)

#will not stop the figure if use this
plt.show()
plt.draw()

#the figure will be closed after 3s
#plt.show(block=False)
#plt.draw()
#time.sleep(3)

plt.close()
