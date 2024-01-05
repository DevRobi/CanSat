from machine import Pin, UART
from time import sleep
from micropyGPS import MicropyGPS
NPNm=Pin(2, Pin.OUT)
NPNp=Pin(13, Pin.OUT)
PNPm=Pin(12, Pin.OUT)
PNPp=Pin(3, Pin.OUT)
PPS=Pin(21, Pin.IN)

def motor_on(dir):
    motor_off()
    if dir == 1:
        NPNp.value(1)
        PNPp.value(0)
    elif dir == -1:
        NPNm.value(1)
        PNPm.value(0)
        
        
def motor_off():
    NPNp.value(0)
    PNPp.value(1)
    NPNm.value(0)
    PNPm.value(1)

def pull_string():
    dir_in = 0.1
    dir_out = 0
    dir_keep = 1
    motor_on(-1)
    sleep(dir_in)
    motor_off()
    sleep(dir_keep)
    '''motor_on(1)
    sleep(dir_out)
    motor_off()'''


pull_string()
led = Pin(25, Pin.OUT)
ledgreen = Pin(15, Pin.OUT)
ledred = Pin(14, Pin.OUT)