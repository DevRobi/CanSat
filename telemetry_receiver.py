import struct
from machine import Pin, UART
import time
import binascii
class measurements_cansat:
    def __init__(self, temperature, altitude, pressure, latitude, longitude, tics_per_sec, timestamp, geiger_vol, battery_vol):
        self.temp = temperature #float 4 bytes
        self.alt = altitude #float 4 bytes
        self.p = pressure #float 4 bytes
        self.lat = latitude #float 4 bytes
        self.long = longitude #float 4 bytes
        self.tics_per_sec = tics_per_sec #int 2 bytes
        self.timestamp = timestamp #int 4 bytes
        self.geiger_vol = geiger_vol #int 2 bytes
        self.battery_vol = battery_vol #int 2 bytes
        
# initialize UART
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), timeout=500)
uart.write(b'sys reset\r\n')
uart.readline()
uart.readline()
uart.readline()
uart.readline()
# set frequency and spreading factor
uart.write(b'radio set freq 867200000\r\n')
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
        uart.readline()
        #print(uart.readline().decode().strip())


        if 'radio_rx' in message:
            message = bytearray(binascii.unhexlify(message[9:]))
            #print(message)
            telemetry = measurements_cansat(0,0,0,0,0,0,0,0,0)
            # change object's values by struct.unpack sent data
            [telemetry.temp, telemetry.alt, telemetry.p, telemetry.lat, telemetry.long,
            telemetry.tics_per_sec, telemetry.timestamp, telemetry.geiger_vol, telemetry.battery_vol] = struct.unpack("fffffhlhh",message)
            # print out data
            print(telemetry.temp, telemetry.alt, telemetry.p, telemetry.lat, telemetry.long,
                  telemetry.tics_per_sec, telemetry.timestamp, telemetry.geiger_vol, telemetry.battery_vol)
            f = open("log.txt", "a")
            f.write(str((telemetry.temp, telemetry.alt, telemetry.p, telemetry.lat, telemetry.long,
                         telemetry.tics_per_sec, telemetry.timestamp, telemetry.geiger_vol, telemetry.battery_vol)))
            f.write("\n")
            f.close()

        else:

            print("error")

        
        #print("Received message: " + message)