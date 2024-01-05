from machine import Pin, I2C, PWM, Timer, UART, SPI
from time import sleep

import math
import binascii

import uos
import struct
from micropyGPS import MicropyGPS
my_gps = MicropyGPS()
PMTK_SET_NMEA_OUTPUT_RMCGGA = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*"
gps = UART(1, 9600, tx=Pin(4), rx=Pin(5),timeout=100)
def gps_send(msg):
    cs=0x00
    for c in msg :
        if c !='$' and c !='*':
            cs = cs ^ ord(c)
    gps_msg = msg + (chr( (cs>>4) + ord('0') )) + chr( (cs&0x0f) + ord('0') ) + '\r\n'
    gps.write(gps_msg)
    for p in msg:
        my_gps.update(p)
    print(gps.readline())
    print(my_gps.longitude)
gps_send(PMTK_SET_NMEA_OUTPUT_RMCGGA)

#telemetry---------------------------------------------
class measurements_cansat:
    def __init__(self, temperature, altitude, pressure, latitude, longitude, tics_per_sec, timestamp, geiger_vol, battery_vol):
        self.temp = temperature #float
        self.alt = altitude #float
        self.p = pressure #float
        self.lat = latitude #float from my_gps.latitude
        self.long = longitude #float from my_gps.longitude
        self.tics_per_sec = tics_per_sec #int
        self.timestamp = timestamp #int from my_gps.timestamp
        self.geiger_vol = geiger_vol #int 2 bytes
        self.battery_vol = battery_vol #int 2 bytes

tel = measurements_cansat(12.3, 12.3, 12.3, 12.3, 12.3, 123, 311001132, 0, 0)

#radio------------------------------------------------
uart = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17), timeout=500)
uart.write(b'sys reset\r\n')
print(uart.readline())
print(uart.readline())
print(uart.readline())
print(uart.readline())
uart.write(b'radio set freq 867200000\r\n')
print(uart.readline())
uart.write(b'radio set sf sf8\r\n')
print(uart.readline())

while True:
    radio_message = bytearray(struct.pack("fffffhlhh", tel.temp, tel.alt, tel.p, tel.lat, tel.long, tel.tics_per_sec, tel.timestamp, tel.geiger_vol, tel.battery_vol))
    uart.write(b'radio tx ' + binascii.hexlify(radio_message) + ' 0\r\n')
    print(uart.readline())
    print(uart.readline())
    print(uart.readline())
    sleep(1)