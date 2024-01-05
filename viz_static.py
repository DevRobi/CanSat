import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import ast
import math

reference_alt = 0 #ujdorogd is around 233m
standardPressure = 1013.25
time = []
alt = []
gps_alt = []
pressure = []
temp = []
clicks = []
batt_vol = []
FILE_TO_READ = 'C:/Users/deven/Documents/cansat/failed_flight.txt'
FROM_TIMESTAMP = 122242
TO_TIMESTAMP = 125429

def convert_timestamp_to_seconds(ts):
    #ts is string

    h = str(ts)[:2]
    m = str(ts)[2:4]
    s = str(ts)[4:]
    print(h, m, s)
    if h[0] == '0':
        h = h[1]
    if m[0] == '0':
        m = m[1]
    if s[0] == '0':
        s = s[1]
    
    h, m, s = int(h)*3600, int(m)*60, int(s)
    return h + m + s
    

def calc_derivative(ls):
    new_ls = []
    for i in range(1, len(ls)):
        new_ls.append(ls[i]-ls[i-1])
    return new_ls

file = open(FILE_TO_READ, 'r')
for line in file.readlines():
    if "bytearray" not in line:
        # Parse the string as a tuple
        tuple_object = ast.literal_eval(line)

# Convert the tuple to a list
        list_object = list(tuple_object)
        TS = int(list_object[0])
        TEMP = list_object[1]
        P = list_object[2]
        ALT = list_object[3]
        TICS = list_object[6]
        BATT_VOL = list_object[8]

        if TS >= FROM_TIMESTAMP and TS <= TO_TIMESTAMP:

            # convert time value to seconds only
            timestamp_seconds = convert_timestamp_to_seconds(TS)
            from_timestamp_seconds = convert_timestamp_to_seconds(FROM_TIMESTAMP)
            time.append(timestamp_seconds - from_timestamp_seconds)
            alt.append(
                (1 - math.pow((P / standardPressure), 0.190284)) * 145366.45 * 0.3048 )
            pressure.append(P)
            temp.append(TEMP)
            clicks.append(TICS)
            batt_vol.append(BATT_VOL)
            gps_alt.append(ALT)

print(pressure)
speed = calc_derivative(alt)
plt.rcParams['figure.figsize'] = [15, 12]
plt.style.use('seaborn')
plt.rcParams["font.family"] = "Cambria"
# Initialise the subplot function using number of rows and columns
fig = plt.figure()
fig.suptitle("Primary and Secondary Mission Measurements")
ax1 = plt.subplot2grid((2, 3), (0, 0))
# ax1.set_aspect('equal')
ax2 = plt.subplot2grid((2, 3), (1, 0))
ax3 = plt.subplot2grid((2, 3), (0, 1))
ax4 = plt.subplot2grid((2, 3), (1, 1))
ax5 = plt.subplot2grid((2,3), (0,2))
ax6 = plt.subplot2grid((2,3), (1,2))
# Altitude vs Time
ax1.scatter(time, alt, c='blue', s=10)
#ax1.plot(time, gps_alt, c='pink', label = 'GPS Alt')
ax1.set_title('Altitude vs Time', fontdict={'fontname':'Cambria'})
ax1.set_ylabel('Altitude [m]', fontdict={'fontname': 'Cambria'})
ax1.tick_params(labelsize=8)
ax1.legend() #legend locations: upper l/r, bottom l/r, center l/r etc. right left center
# Pressure vs Time
ax2.scatter(time, pressure, c='green', s=10)
#ax2.plot(time, pressure, c='green')
ax2.set_title('Pressure vs Time', fontdict={'fontname': 'Cambria'})
ax2.set_xlabel('Time [s]', fontdict={'fontname': 'Cambria'})
ax2.set_ylabel('Pressure [hPa]', fontdict={'fontname': 'Cambria'})
ax2.tick_params(labelsize=8)
ax2.set_ylim(980, 1020)
# Time vs Temperature
ax3.scatter(time, temp, c='navy', s=10)
#ax3.plot(time, temp, c='navy')
ax3.set_title('Temperature vs Time', fontdict={'fontname': 'Cambria'})
ax3.set_ylabel('Temperature [C]', fontdict={'fontname': 'Cambria'})
ax3.tick_params(labelsize=8)

# Tics vs Altitude
ax4.scatter(time, clicks, c='darkblue', s=10)
#ax4.plot(time, clicks, c='darkblue')
ax4.set_title('Total Tics vs Time', fontdict={'fontname': 'Cambria'})
ax4.set_xlabel('Time [s]', fontdict={'fontname': 'Cambria'})
ax4.set_ylabel('Total Tics [total # of tics]',
               fontdict={'fontname': 'Cambria'})
ax4.tick_params(labelsize=8)
# Speed vs Time
ax5.scatter(time[1:], speed, c = 'red', s=10)
#ax5.plot(time[1:], speed, c = 'red')
ax5.set_title('Speed vs Time', fontdict={'fontname': 'Cambria'})
ax5.set_ylabel('Speed [m/s]', fontdict={'fontname': 'Cambria'})
ax5.tick_params(labelsize=8)
# Battery voltage
ax6.scatter(time, batt_vol, c='orange', s=10)
#ax6.plot(time, batt_vol, c='orange')
ax6.set_title('Battery Voltage vs Time', fontdict={'fontname': 'Cambria'})
ax6.set_ylabel('Voltage [mV]', fontdict={'fontname': 'Cambria'})
ax6.set_xlabel('Time [s]', fontdict={'fontname': 'Cambria'})
ax6.set_ylim(3900,4000)
ax6.tick_params(labelsize=8)

# Combine all the operations and display
plt.show()
