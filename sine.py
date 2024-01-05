#from machine import Pin, PWM
import time, math
from time import sleep
from matplotlib import pyplot as plt
#pwm0 = PWM(Pin(25))      # create PWM object from a pin
#pwm0.duty_u16(0)
u = []
def sin_values(k,fs):
    f = 1000
    for i in range(0, k+1):
        value = math.sin(2*math.pi*f*i/fs)
        adjusted_val = (value+1)*65535/2
        u.append(adjusted_val)
sin_values(2000,80000)

'''plt.xlabel("Number of samples", fontsize=20); plt.ylabel("Duty cycle", fontsize=20)
plt.xticks(fontsize=14);plt.yticks(fontsize=14)
plt.plot(u);plt.show()

while True:
    for v in u:
        print(v)
        pwm0.duty_u16(int(v))
        time.sleep(0.1)'''
#pwm0.duty_u16()         # get current duty cycle, range 0-65535
#wm0.duty_u16(200)      # set duty cycle, range 0-65535
#pwm0.deinit()           # turn off PWM on the pin
#pwm0.freq()             # get current frequency