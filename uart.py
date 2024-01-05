from machine import UART, Pin
import time
#writing from laptop to pico, reading it with pico and writing back to laptopuart1 = UART(01, baudrate=9600, tx=Pin(4), rx=Pin(5))
#meanwhile write from terminal command: miniterm /dev/ttyUSB0 9600
while True:
    if uart1.any()>0:
        x = uart1.read().decode() #pico reads incoming data from laptop
        if x == 't':
            uart1.write('Transmit\n') #write back to laptop based on the incoming data
        elif x == 'r':
            uart1.write('Receive\n')
        else:
            uart1.write("Unknown command\n")
        print(x)