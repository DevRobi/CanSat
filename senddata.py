from machine import Pin, UART
import time
import binascii

n = 1
text = "hello cansat"
message = ""

# initialize UART
uart = UART(0, baudrate=125000, tx=Pin(16), rx=Pin(17), timeout=500)

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

while True:
    # send a message every 5 second
    
    message = text + " " + str(n)
    print(message)
    
    uart.write(b'radio tx ' + binascii.hexlify(message) + ' 0\r\n')
    
    uart.readline()
    uart.readline()
    uart.readline()

    n += 1
    
    time.sleep(1)