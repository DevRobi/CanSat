# importing libraries
import matplotlib.pyplot as plt
import numpy as np
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import ast
import math
from time import sleep
from threading import Thread

NUM_POINTS = 20
FILE_TO_READ = 'C:/Users/deven/Documents/cansat/Pico/MainScript/log.txt'
#'C:/Users/deven/Documents/cansat/Pico/MainScript/log.txt'
#'C:/Users/deven/Documents/cansat/Pico/MainScript/log.txt'
FROM_TIMESTAMP = 112750
TO_TIMESTAMP = 112922
standardPressure = 1013.25
prev = 0
current = 0
data_points = np.zeros((NUM_POINTS, 6))
time = [0]
alt = [0]
pressure = [0]
temp = [0]
clicks = [0]
batt_vol = [0]
n = 0
t_start = 0
fig = plt.figure()
fig.suptitle("")
plt.rcParams['figure.figsize'] = [15, 12]
plt.style.use('seaborn')
plt.rcParams["font.family"] = "Cambria"
ax1 = plt.subplot2grid((2, 3), (0, 0))
# ax1.set_aspect('equal')
ax2 = plt.subplot2grid((2, 3), (1, 0))
ax3 = plt.subplot2grid((2, 3), (0, 1))
ax4 = plt.subplot2grid((2, 3), (1, 1))
ax5 = plt.subplot2grid((2,3), (0,2))
ax6 = plt.subplot2grid((2,3), (1,2))
# Initialise the subplot function using number of rows and columns
ax1.set_title('Altitude vs Time')
#ax1.set_xlabel('Time / s', fontsize=8)
ax1.set_ylabel('Altitude / [m]')
ax2.set_title('Pressure vs Time')
ax2.set_xlabel('Time / [s]')
ax2.set_ylabel('Pressure / [hPa]')
ax3.set_title('Temperature vs Time')
#ax3.set_xlabel('Time / m')
ax3.set_ylabel('Temperature / [C]')
##simulate geiger
ax4.set_xlabel('Time / [s]')
ax4.set_ylabel('Total Ticks')
###speed
ax5.set_ylabel('Speed / [m/s]')
##voltage of battery
ax6.set_ylabel('Battery voltage')
ax6.set_xlabel('Time / [s]')

def convert_timestamp_to_seconds(ts):
    # ts is int
    if len(str(ts)) == 5:
        ts = '0'+ str(ts)

    h = str(ts)[:2]
    m = str(ts)[2:4]
    s = str(ts)[4:]

    if h[0] == '0':
        h = h[1]
    if m[0] == '0':
        m = m[1]
    if s[0] == '0':
        s = s[1]

    h, m, s = int(h)*3600, int(m)*60, int(s)
    return h + m + s

def data_gen():
    global data_points, n   
    data = [alt[n], pressure[n], temp[n], time[n], clicks[n], batt_vol[n]]
    data_points = np.roll(data_points, -1, axis=0)
    data_points[-1, :] = data
    #print(data)
    #print(data_points)
    n += 1
    yield data_points


def init():
    ax1.cla()
    ax2.cla()
    ax3.cla()
    ax4.cla()
    limits = 200
    #ax1.set_xlim(0, 120)
    ax1.set_ylim(50,70)

    #ax2.set_xlim(0, 120)
    ax2.set_ylim(900, 1100)

    #ax3.set_xlim(0, 180)
    ax3.set_ylim(23.8, 25.2)

def run(data):
    global time, alt, pressure, temp, n, t_start
    
    file = open(FILE_TO_READ, 'r')
    last_line = file.readlines()[-1]
    
    #file.close()
    tuple_object = ast.literal_eval(last_line)
    list_object = list(tuple_object)
    if t_start == 0:
        t_start = list_object[0]
    
    # ############################convert time value to seconds only -- real data
    timestamp_seconds = convert_timestamp_to_seconds(list_object[0])
    from_timestamp_seconds = convert_timestamp_to_seconds(t_start)
    t = timestamp_seconds-from_timestamp_seconds
    print(t)
    time.append(t) #timestamp_seconds - from_timestamp_seconds

    ################demo data#################
    #time.append(n)
    ###############demo data##################
    alt.append(
        (1 - math.pow((list_object[2] / standardPressure), 0.190284)) * 145366.45 * 0.3048)
    #alt.append(list_object[1])
    pressure.append(list_object[2])
    temp.append(list_object[1])
    clicks.append(list_object[6])
    batt_vol.append(list_object[8])
    # Alt vs Time
    ax1.cla()
    ax1.scatter(data_points[:, 3], data_points[:, 0], c='blue', s=10)
    ax1.plot(data_points[:, 3], data_points[:, 0], c='blue')
    # P vs Time
    ax2.cla()
    ax2.scatter(data_points[:, 3], data_points[:, 1], c='green', s=10)
    ax2.plot(data_points[:, 3], data_points[:, 1], c='green')
    ax2.set_ylim(900, 1100)
    # Temp vs Time
    ax3.cla()
    ax3.scatter(data_points[:, 3], data_points[:, 2], c = 'navy', s=10)
    ax3.plot(data_points[:, 3], data_points[:, 2], c='navy')
    #Clicks vs Time
    ax4.cla()
    ax4.scatter(data_points[:, 3], data_points[:, 4], c='darkblue', s=10)
    ax4.plot(data_points[:, 3], data_points[:, 4], c='darkblue')
    #Speed
    ax5.cla()
    ax5.plot(data_points[:, 3][1:], calc_derivative(data_points[:, 0]), c='red')
    #Batt vol
    ax6.cla()
    ax6.plot(data_points[:, 3], data_points[:, 5], c='orange')

    fig.suptitle("Motor State: " + str(list_object[-2]))
    #reset axes labels, titles after clearing axes
    ax1.set_title('Alt vs T')
    ax1.set_ylabel('Alt / [m]')

    ax2.set_title('P vs T')
    ax2.set_xlabel('T / [s]')
    ax2.set_ylabel('P / [hPa]')

    ax3.set_title('Temp vs T')
    ax3.set_ylabel('Temp / [C]')

    ax4.set_title('Tics vs T')
    ax4.set_xlabel('T / [s]')
    ax4.set_ylabel('# Tics')

    ax5.set_title('Speed vs T')
    ax5.set_ylabel('v / [m/s]')

    ax6.set_title('Batt Vol vs T')
    ax6.set_ylabel("Voltage / [V]")
    ax6.set_xlabel('T / [s]')


'''file = open(FILE_TO_READ, 'r')
last_line = file.readlines()[-1]
# file.close()
tuple_object = ast.literal_eval(last_line)
list_object = list(tuple_object)
if t_start == 0:
    t_start = list_object[0]

# convert time value to seconds only
print(list_object)
timestamp_seconds = convert_timestamp_to_seconds(list_object[0])
from_timestamp_seconds = convert_timestamp_to_seconds(t_start)
t = timestamp_seconds-from_timestamp_seconds
#time.append(n)  # timestamp_seconds - from_timestamp_seconds
print(list_object[0])
alt.append(
    (1 - math.pow((list_object[2] / standardPressure), 0.190284)) * 145366.45 * 0.3048)
# alt.append(list_object[1])
pressure.append(list_object[2])
temp.append(list_object[1])
clicks.append(list_object[6])
print(temp, clicks, pressure, alt)'''


def calc_derivative(ls):
    new_ls = []
    for i in range(1, len(ls)):
        new_ls.append(ls[i]-ls[i-1])
    return new_ls

ani = animation.FuncAnimation(fig, run, data_gen, init_func=init, interval=500, cache_frame_data=False)

# Show the plot
plt.show()  
