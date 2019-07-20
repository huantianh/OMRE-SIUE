import matplotlib.pyplot as plt
import numpy as np

#file = open("reference_pwm.txt",'w')

#x = np.arange(0,55.1,.1)
#n = len(x)

#y = np.zeros(n)

#y[x<5] = 0
#y[x>=5] = 50
#y[x>=10] = 100
#y[x>=15] = 150
#y[x>=20] = 200
#y[x>=25] = 250
#y[x>=30] = 200
#y[x>=35] = 150
#y[x>=40] = 100
#y[x>=45] = 50
#y[x>=50] = 0


#for idx, item in enumerate(x):
	
	#file.writelines(str(x[idx])+','+str(y[idx])+'\n')

#file.close

x = []
y = []

f = open("reference_pwm.txt",'r')
lines = f.readlines()
for line in lines:
	#print(line.split(',')[0])
	print(line.split(',')[1])	
	x.append(line.split(',')[0])
	y.append(line.split(',')[1])
	

#plt.plot(x,y, 'b', linewidth=2)
#plt.show()
#plt.close()
