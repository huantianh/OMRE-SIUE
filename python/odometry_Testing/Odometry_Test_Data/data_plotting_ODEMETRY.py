import matplotlib.pyplot as plt
import csv,os,time
import numpy as np 


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
	for row in plots:
			x.append(float(row[0]))
			y1.append(float(row[1]))
			y2.append(float(row[2]))
			y3.append(float(row[3]))
		
fig = plt.plot(x,y1, 'b', label='Encoder', linewidth=3)
fig = plt.plot(y3,y1, 'r', label='IMU', linewidth=3)
plt.legend()
plt.autoscale(enable=True, axis='x', tight=True)
plt.xlabel('X_axis_Moved')
plt.ylabel('Y_axis_Moved')
plt.title('Encoder_Pos')

##Save image into full size 
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
