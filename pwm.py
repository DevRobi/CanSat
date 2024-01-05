from machine import Pin, PWM
from time import sleep
pwm = PWM(Pin(25))
pwm.freq(1000)
x = 0
direction = 1
while x < 1000000:
 for i in range(65025):
    x += direction
    if x > 65024:
        x = 65024
        direction = -1
    elif x < 0:
        x = 0
        direction = 1
    pwm.duty_u16(0)
    sleep(0.0001)