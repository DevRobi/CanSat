from math import sin,pi
fs = 8000
f = 200
samples=[]
for i in range(int(fs/f)):
	s =  int(65535*(sin(i*2*pi/n)+1)/2) 
	samples.append(s)
	prin(i,s)
	
# print(samples)