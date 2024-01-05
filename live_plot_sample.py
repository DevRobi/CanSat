import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from time import sleep


# Define the number of data points to keep track of
NUM_POINTS = 200
data_points = np.zeros((NUM_POINTS, 3))
# Initialize the plot
fig = plt.figure()
ax1 = plt.subplot2grid((2,2),(0,0))
#ax1.set_aspect('equal')
ax2 = plt.subplot2grid((2,2),(1,0))
ax3 = plt.subplot2grid((2, 2), (0, 1))
ax4 = plt.subplot2grid((2,2), (1,1))

def data_gen():
	global data_points
	data = [1,2,3]


	data_points = np.roll(data_points, -1, axis=0)
	data_points[-1, :] = data
	yield data_points


def init():
	ax1.cla()
	limits=200
	ax1.set_xlim(-5,5)
	ax1.set_ylim(-5,5)

def run(data):
	print(data)
	# update the data
	# Draw the line connecting the data points
	ax1.plot(data_points[:, 0], data_points[:, 2])
   
	ax2.scatter(data_points[:, 0], data_points[:, 1])
	
	
	ax1.set_title("példa ábra")
	#ax1.grid()
	#ax1.figure.canvas.draw()


# Only save last 100 frames, but run forever
ani = animation.FuncAnimation(fig, run, data_gen, interval=500, init_func=init,cache_frame_data=False)						   
plt.show()