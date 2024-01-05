from machine import Pin, I2C, PWM, Timer, UART, SPI
from time import sleep
import BME280
import math
import binascii
import sdcard
import uos
import struct

tim = Timer()

#BMP-----------------------------------------------------
standardPressure = 1013.25 #in hPa
pressureLapseRate = 1/9.14 #hPa/m

sda = Pin(12)
scl = Pin(13)
i2c = I2C(0, sda=sda, scl=scl, freq=400000)
bme = BME280.BME280(i2c=i2c)

temperature = 0
pressure = 0
altitude = 0

#geiger--------------------------------------------------

PWM_pin = PWM(Pin(22))
INPUT_pin = Pin(20, Pin.IN)

dutyCycle = 0
frequency = 30000

ticCount = 0
numSamples = 10
samples = [0] * numSamples
sampleIndex = 0
ticsPerSecond = 0

PWM_pin.freq(frequency)
PWM_pin.duty_u16(100-dutyCycle)

#sd-----------------------------------------------------

cs = Pin(9, Pin.OUT)

spi = SPI(0,
            baudrate=1000000,
            polarity=0,
            phase=0,
            bits=8,
            firstbit=SPI.MSB,
            sck=Pin(6),
            mosi=Pin(7),
            miso=Pin(8))

sd = sdcard.SDCard(spi, cs)
#mount sd
vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")

#gps
latitude = 0.0
longitude = 0.0

#telemetry---------------------------------------------
class measurements_cansat:
    def __init__(self, temperature, altitude, pressure, latitude, longitude, tics_per_sec, timestamp):
        self.temp = temperature #float
        self.alt = altitude #float
        self.p = pressure #float
        self.lat = latitude #float
        self.long = longitude #float
        self.tics_per_sec = tics_per_sec #int
        self.timestamp = timestamp #int

tel = measurements_cansat(12.3, 12.3, 12.3, 12.3, 12.3, 123, 3110301130)

#radio------------------------------------------------
uart = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17), timeout=500)
uart.write(b'sys reset\r\n')
print(uart.readline())
print(uart.readline())
print(uart.readline())
print(uart.readline())
uart.write(b'radio set freq 868000000\r\n')
print(uart.readline())
uart.write(b'radio set sf sf8\r\n')
print(uart.readline())

def Main(t):
    global ticCount, sampleIndex, ticsPerSecond, temperature, pressure, altitude, bme, sd
    global pressureLapseRate, standardPressure, latitude, longitude, tel, uart
    #BMP
    
    temperature = float(bme.temperature[:-3])
    pressure = float(bme.pressure[:-3])
    altitude = (1 - math.pow((pres / standardPressure), 0.190284)) * 145366.45 * 0.3048
    '''
    #geiger    
    samples[sampleIndex] = ticCount
    sampleIndex = (sampleIndex + 1) % numSamples
    
    sum = 0
    for sample in samples:
            sum += sample
    
    ticsPerSecond = int(round(sum / numSamples))
    '''
    #gps
    
    #processing
    tel.temp = temperature
    tel.p = pressure
    tel.alt = altitude
    tel.tics_per_sec = ticsPerSecond
    tel.lat = latitude
    tel.long = longitude
    tel.timestamp = timestamp
    
    with open("/sd/telemetry.txt", "a") as file:
        file.write(str(tel.temp) + " " + str(tel.alt) + " " + str(tel.p) + " " + str(tel.lat) + " " + str(tel.long) + " " + str(tel.tics_per_sec) + " " + str(tel.timestamp))
    
    radio_message = bytearray(struct.pack("fffffhl", tel.temp, tel.alt, tel.p, tel.lat, tel.long, tel.tics_per_sec, tel.timestamp))
    uart.write(b'radio tx ' + binascii.hexlify(radio_message) + ' 0\r\n')
    uart.readline()
    uart.readline()
    uart.readline()
    
def Tic():
    global ticCount
    ticCount += 1
    
INPUT_pin.irq(handler=Tic, trigger=Pin.IRQ_RISING) #trigger

tim.init(period=1000, callback=Main) #timer that handles everything

while True:
    if uart.any():
        if 'fre' in uart.readline():
            # fre,freq,baud,sf
            uart.write(b'radio set freq (freq)\r\n')
            print(uart.readline())
            uart.write(b'radio set sf sf(sf)\r\n')
            print(uart.readline())
            uart = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17), timeout=500)
        elif 'stp' in uart.readline(): #stop
            pass
        elif 'str' in uart.readline(): #start
            pass