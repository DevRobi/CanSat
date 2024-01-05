from machine import Pin, UART, Timer
import machine
import time
import binascii

uart = UART(0, baudrate=125000, tx=Pin(0), rx=Pin(1), timeout=500)

uart.write(b'sys reset\r\n')
print(uart.readline())
print(uart.readline())
print(uart.readline())
print(uart.readline())
# set frequency and spreading factor
uart.write(b'radio set freq 868000000\r\n')
print(uart.readline())
uart.write(b'radio set sf sf8\r\n')
print(uart.readline())

tim = Timer()

def Main(t):
    message = "hello cansat"
    
    uart.write(b'radio tx ' + binascii.hexlify(message) + ' 0\r\n')
    print("hello")
    '''
    uart.readline()
    uart.readline()
    uart.readline()
    '''

tim.init(period=1000, callback=Main)

while True:
    if(uart.any()):
        print(uart.readline())