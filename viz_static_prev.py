import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import ast
import math

standardPressure = 1013.25
time = []
alt = []
pressure = []
temp = []
FILE_TO_READ = 'C:/Users/deven/Documents/cansat/Pico/MainScript/log.txt'
FROM_TIMESTAMP = 104430
TO_TIMESTAMP = 104505

def convert_timestamp_to_seconds(ts):
    #ts is string

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
    


file = open(FILE_TO_READ, 'r')
for line in file.readlines():
    if "bytearray" not in line:
        # Parse the string as a tuple
        tuple_object = ast.literal_eval(line)

# Convert the tuple to a list
        list_object = list(tuple_object)

        if int(list_object[6]) >= FROM_TIMESTAMP and int(list_object[6]) <= TO_TIMESTAMP:

            # convert time value to seconds only
            timestamp_seconds = convert_timestamp_to_seconds(int(list_object[6]))
            from_timestamp_seconds = convert_timestamp_to_seconds(FROM_TIMESTAMP)
            time.append(timestamp_seconds - from_timestamp_seconds)
            alt.append(
                (1 - math.pow((list_object[2] / standardPressure), 0.190284)) * 145366.45 * 0.3048 + 100)
            pressure.append(list_object[2])
            temp.append(list_object[0])

def calc_derivative(ls):
    new_ls = []
    for i in range(1, len(ls)-1):
        new_ls.append(ls[i]-ls[i-1])
    return new_ls

plt.rcParams['figure.figsize'] = [15, 12]

# simulate geiger
# Sample data for altitude
altitudes = np.linspace(0, 1000, 50)

# Create quadratic total ticks data based on altitude with random error
total_ticks = (altitudes**2 * 0.001 + altitudes * 0.5 +
               np.random.normal(loc=0, scale=50, size=50)) / 10

# Initialise the subplot function using number of rows and columns
fig = plt.figure()
ax1 = plt.subplot2grid((2, 2), (0, 0))
# ax1.set_aspect('equal')
ax2 = plt.subplot2grid((2, 2), (1, 0))
ax3 = plt.subplot2grid((2, 2), (0, 1))
ax4 = plt.subplot2grid((2, 2), (1, 1))
print(len(time[2:]), len(calc_derivative(alt)))
# Altitude vs Time
ax1.scatter(time, alt)
ax1.set_title('Time vs Altitude')
#ax1.set_xlabel('Time / s')
ax1.set_ylabel('Altitude / m')
# Pressure vs Time
ax2.plot(time[2:], calc_derivative(alt))
ax2.set_title('Time vs Derivative of Altitude')
ax2.set_xlabel('Time / s')
ax2.set_ylabel('dA/dT Speed')
# Altitude vs Temperature
ax3.scatter(alt, temp)
ax3.set_title('Altitude vs Temperature')
ax3.set_xlabel('Altitude / m')
ax3.set_ylabel('Temperature / C')
# Tics vs Altitude
ax4.scatter(altitudes, total_ticks)
ax4.set_title('Altitude vs Total Ticks [SIMULATED ONLY]')
ax4.set_xlabel('Altitude / m')
ax4.set_ylabel('Total Ticks')

# Combine all the operations and display
plt.show()
