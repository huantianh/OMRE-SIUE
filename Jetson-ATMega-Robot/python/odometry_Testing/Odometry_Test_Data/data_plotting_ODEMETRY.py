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
y = []

with open(root.filename,'r') as csvfile:
	plots = csv.reader(csvfile, delimiter=',')
	for row in plots:
			x.append(float(row[0]))
			y.append(float(row[1]))
		

plt.plot(x,y,label='Loaded from file!')
plt.xlabel('X')
plt.ylabel('Y')
plt.title(my_txt1)
plt.savefig(my_txt1+'.png')

#will not stop the figure if use this
#plt.show()
#plt.draw()

#the figure will be closed after 3s
plt.show(block=False)
plt.draw()
time.sleep(3)
plt.close()
