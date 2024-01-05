from machine import Pin, UART
import time
import binascii

# initialize UART
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), timeout=500)
uart.write(b'sys reset\r\n')
uart.readline()
uart.readline()
uart.readline()
uart.readline()
# set frequency and spreading factor
uart.write(b'radio set freq 868000000\r\n')
print(uart.readline())
uart.write(b'radio set sf sf8\r\n')
print(uart.readline())

# start listening for incoming messages
uart.write(b'radio rx 0\r\n')
print(uart.readline())

while True:
    # check if a message has been received
    if uart.any():
        # read the incoming message
        message = uart.readline().decode('utf-8').strip()
        
        uart.write(b'radio get pktrssi\r\n')
        print(uart.readline().decode().strip())

        try:
            if 'radio_rx' in message:
                message = binascii.unhexlify(message[9:]).decode('utf-8')

                print(message)

            else:

                print(message)

        except:
            print(message)
        #print("Received message: " + message)