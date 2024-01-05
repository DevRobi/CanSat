from machine import Pin, I2C, PWM, Timer, UART, SPI, ADC
from time import sleep
import BME280
import math
import binascii
import sdcard
import uos
import struct
from micropyGPS import MicropyGPS

tim = Timer()
radioOff = Timer()

led = Pin(25, Pin.OUT)
gps = UART(1, 9600, tx=Pin(4), rx=Pin(5),timeout=100)

cycleCount = 0
measureFlag = 0
parachute = 0
motor_on_time = 0
PULL_TIME = 18000
HOLD_TIME = 2000
prev_state = 0

#geiger--------------------------------------------------
PWM_pin = PWM(Pin(22))
INPUT_pin = Pin(20, Pin.IN)
ALERTON_led = Pin(7, Pin.OUT)
ALERTON_led.value(1)
ALERTOFF_led = Pin(6, Pin.OUT)

dutyCycle = 0
frequency = 30000

ticCount = 0
numSamples = 10
samples = [0] * numSamples
sampleIndex = 0
ticsPerSecond = 0

def GeigerDuty(percentDuty):
    return int(65535 - (65535 * (percentDuty / 100)))

PWM_pin.freq(frequency)
PWM_pin.duty_u16(GeigerDuty(dutyCycle))

geigerADC = ADC(Pin(26))
geigerVol = 0

#battery-------------------------------------------------
batteryADC = ADC(Pin(28))
batterVol = 0

#BMP-----------------------------------------------------

standardPressure = 1013.25 #in hPa
pressureLapseRate = 1/9.14 #hPa/m

bmpOk = 1

sda = Pin(10)
scl = Pin(11)
i2c = I2C(1, sda=sda, scl=scl, freq=400000)
try:
    bme = BME280.BME280(i2c=i2c)
except:
    print("no bmp")
    bmpOk = 0
else:
    print("bmp ok")
    bmpOk = 1

temperature = 0
pressure = 0
presAltitude = 0

#sd-----------------------------------------------------
cs = Pin(9, Pin.OUT)

spi = SPI(1,
            baudrate=1000000,
            polarity=0,
            phase=0,
            bits=8,
            firstbit=SPI.MSB,
            sck=Pin(14),
            mosi=Pin(15),
            miso=Pin(8))

sdOk = 1

try:
    sd = sdcard.SDCard(spi, cs)
    
    vfs = uos.VfsFat(sd)
    uos.mount(vfs, "/sd")
except:
    sdOk = 0
    print("no sd card")
else:
    sdOk = 1
    print("sd card ok")

#gps
altitude = 0.0
latitude = 0.0
longitude = 0.0
timestamp = 123456789
my_gps = MicropyGPS(2) # 1 hour offset (UTC+1)

PMTK_SET_NMEA_OUTPUT_RMCGGA = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*"
def gps_send(msg):
    cs=0x00
    for c in msg :
        if c !='$' and c !='*':
            cs = cs ^ ord(c)
    gps_msg = msg + (chr( (cs>>4) + ord('0') )) + chr( (cs&0x0f) + ord('0') ) + '\r\n'
    gps.write(gps_msg)
gps_send(PMTK_SET_NMEA_OUTPUT_RMCGGA)

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
    dir_in = 5
    dir_out = 5
    dir_keep = 2
    motor_on(1)
    #sleep(dir_in)
    motor_off()
    #sleep(dir_keep)
    motor_on(-1)
    #sleep(dir_out)
    motor_off()
    
def statemachine_parachute(state):
    global cycleCount
    global motor_on_time
    
    if state == 0:
        return 0
    elif state == 1: # pull
        motor_on(1)
        motor_on_time = cycleCount
        print('Pull')
        return 2
    
    elif state == 2: # pulling on
        if motor_on_time + PULL_TIME <= cycleCount:
            motor_off()
            print('motor off')
            return 3
        
    elif state == 3: #hold on
        
        if motor_on_time + PULL_TIME + HOLD_TIME <= cycleCount:
            motor_on(-1)
            print('release')
            return 4
        
    elif state == 4: #release
        if motor_on_time + PULL_TIME + HOLD_TIME + PULL_TIME <= cycleCount:
            motor_off()
            print('motor off')
            return 0
    
    #print(f'no change{state}')
    return state

#telemetry---------------------------------------------
class measurements_cansat:
    def __init__(self, temperature, altitude, pressure, latitude, longitude, tics_per_sec, timestamp, geiger_vol, battery_vol):
        self.temp = temperature #float
        self.alt = altitude #float
        self.p = pressure #float
        self.lat = latitude #float
        self.long = longitude #float
        self.tics_per_sec = tics_per_sec #int
        self.timestamp = timestamp #int
        self.geiger_vol = geiger_vol #int 2 bytes
        self.battery_vol = battery_vol #int 2 bytes

tel = measurements_cansat(12.3, 12.3, 12.3, 12.3, 12.3, 123, 3110301130, 0, 0)

#radio------------------------------------------------
radioOk = 1
status = b''

uart = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17), timeout=500)
print("Wait until boot...")
sleep(1)
print("Radio init...")
n = uart.any()
print(uart.read(n))
print("Radio reset")
uart.write(b'sys reset\r\n')
print(uart.readline())
print(uart.readline())
print(uart.readline())
print(uart.readline())
sleep(2)

print("Radio set freq")
uart.write(b'radio set freq 867900000\r\n')
print(uart.readline())
print("Radio set sf")
uart.write(b'radio set sf sf7\r\n')
print(uart.readline())
print("Radio set power")
uart.write(b'radio set pa on\r\n')
print(uart.readline())
uart.write(b'radio set pwr 20\r\n')

def Blink(t):
    global led
    led.value((led.value() - 1) * -1)

if "ok" in str(uart.readline()):
    radioOk = 1
    status += b'booted,'
    if sdOk == 1:
        status += b'sdOk,'
    else:
        status += b'sdErr,'
    if bmpOk == 1:
        status += b'bmpOk'
    else:
        status += b'bmpErr'
    uart.write(b'radio tx ' + binascii.hexlify(status) + b' 0\r\n')
    uart.readline()
    uart.readline()
    uart.readline()
else:
    radioOk = 0
    radioOff.init(period=200, callback=Blink)

print("DONE")
#uart.write(b'radio rx 0\r\n')
#print(uart.readline())

#falling states---------------------------------------------
fallingState = 0 #0:launching 1:falling (para open) 2:falling (para closed) 3:landing (para open)
valuesInLine = 0
prevAlt = 0
fallStart = 0
fallTime = 5000
openHeight = 500
def StateMachineFalling(state):
    global altitude, presAltitude, valuesInLine, prevAlt, cycleCount, fallStart, fallTime, openHeight
    if state == 0:
        if valuesInLine >= 4:
            valuesInLine = 0
            fallStart = cycleCount
            return 1
        elif prevAlt > presAltitude: 
            valuesInLine += 1
            return state
        else:
            valuesInLine = 0
            return state
    elif state == 1:
        if fallStart + fallTime <= cycleCount:
             #pull in para
             return 2
        else:
            return state
    elif state == 2:
        if my_gps.fix_stat > 0:
            if ((altitude + presAltitude) / 2) <= openHeight:
                #open para
                return 3
            else:
                return state
        else:
            if presAltitude <= openHeight:
                #open para
                return 3
            else:
                return state
        

def Measure():
    global ticCount, sampleIndex, ticsPerSecond, temperature, pressure, altitude, bme, sd
    global pressureLapseRate, standardPressure, latitude, longitude, tel, uart, timestamp, cycleCount
    global my_gps, presAltitude, batteryVol, batteryADC, geigerADC, geigerVol
    #BMP
    
    try:
        
        temperature = bme.read_temperature() / 100
        pressure = bme.read_pressure() / 25600
        #print(temperature, pressure)
        
    except:
        temperature = 0
        pressure = 0
    presAltitude = (1 - math.pow((pressure / standardPressure), 0.190284)) * 145366.45 * 0.3048
    
    '''
    #geiger    
    samples[sampleIndex] = ticCount
    sampleIndex = (sampleIndex + 1) % numSamples
    
    sum = 0
    for sample in samples:
            sum += sample
    
    ticsPerSecond = int(round(sum / numSamples))
    
    ticcount = 0
    '''
    
    #extract lat, long
    long = my_gps.longitude
    lat = my_gps.latitude
    altitude = my_gps.altitude
    longitude = long[0] + long[1] / 60
    latitude = lat[0] + lat[1] / 60
    
    #extract timestamp
    gps_timestamp = my_gps.timestamp
    print(gps_timestamp)
    str_timestamp = ""
    #timestamp to right format
    for x in gps_timestamp: # tuple
        x = int(x)
        if x < 10:
            x = str("0{0}".format(x)) # paddle with zero
        else:
            x = str(x)
        str_timestamp += x
    #convert to int -> final timestamp
    #print(str_timestamp)
    timestamp = int(str_timestamp)
    #voltages
    batteryVol = int(batteryADC.read_u16() * 2 * 2048 / 65535)
    geigerVol = int(geigerADC.read_u16() * 148 * 2048 / 65535 / 1000)
    
    print(batteryVol, geigerVol)
    
    #processing
    tel.temp = temperature
    tel.p = pressure
    tel.alt = altitude
    tel.tics_per_sec = ticsPerSecond
    tel.lat = latitude
    tel.long = longitude
    tel.timestamp = timestamp
    tel.geiger_vol = geigerVol
    tel.battery_vol = batteryVol
    
#     print("GPS:")
#     print(latitude)
#     print(longitude)
#     print(altitude)
#     print(timestamp)
    
    if sdOk == 1:
        with open("/sd/telemetry.txt", "a") as f:
            f.write(str(tel.timestamp) + "," +
                    str(tel.temp) + "," +
                    str(tel.alt) + "," +
                    str(tel.p) + "," +
                    str(tel.lat) + "," +
                    str(tel.long) + "," +
                    str(tel.tics_per_sec) + "," +
                    str(tel.timestamp) + ","+
                    str(tel.geiger_vol) + "," +
                    str(tel.battery_vol) + "\n")
    
    if radioOk == 1:
        led.value(1)
        radio_message = b'SPTM' + bytearray(struct.pack("fffffhlhh", tel.temp, tel.alt, tel.p, tel.lat, tel.long, tel.tics_per_sec, tel.timestamp, tel.geiger_vol, tel.battery_vol))
        uart.write(b'radio rxstop\r\n')
        uart.readline()
        sleep(0.05)
        
        uart.write(b'radio tx ' + binascii.hexlify(radio_message) + b' 0\r\n')
        uart.readline()
        uart.readline()
        uart.readline()
        
        led.value(0)
    
    #cycleCount = 0
    
    uart.write(b'radio rx 900\r\n')
    uart.readline()
    
    
def Tic():
    global ticCount
    ticCount += 1

def Counter(t):
    global cycleCount, measureFlag
    cycleCount += 1
    if cycleCount % 1000 == 0:
        measureFlag = 1

INPUT_pin.irq(handler=Tic, trigger=Pin.IRQ_RISING) #trigger

tim.init(period=1, callback=Counter) #timer that handles everything

while True:
    if geigerVol > 50:
        ALERTON_led.value(1)
        ALERTOFF_led.value(0)
    else:
        ALERTOFF_led.value(1)
        ALERTON_led.value(0)
    
    if parachute != 0:
        parachute = statemachine_parachute(parachute)
    
    if measureFlag == 1:
        print("Measure", cycleCount)
        measureFlag = 0
        Measure()
    
    if gps.any():
        gpgga = gps.readline()
        print("GPGGA")
        print(gpgga)
        for x in str(gpgga):
            my_gps.update(x)
    
    if uart.any():
        message = uart.readline().decode('utf-8').strip()
        if 'radio_rx' in message:
            message = binascii.unhexlify(message[9:]).decode('utf-8')
            print(message)
            
            if 'fre' in message:
                # fre,freq,baud,sf
                freqData = message.split(",")
                
                if "def" not in freqData[1]:
                    uart.write(b'radio set freq ' + freqData[1][1:] + '\r\n')
                    print(uart.readline())
                    print("freqdone")
                
                if "def" not in freqData[2]:
                    uart.write(b'radio set bw ' + freqData[2] + '\r\n')
                    print(uart.readline())
                    print("done")
                
                if "def" not in freqData[3]:
                    uart.write(b'radio set sf sf' + freqData[3][:-1] + '\r\n')
                    print(uart.readline())
                    print("sfdone")
                sleep(0.1)
                uart.write(b'radio tx ' + binascii.hexlify('ack') + b' 0\r\n')
                uart.readline()
                uart.readline()
                uart.readline()
                
            elif 'stp' in message: #stop
                PWM_pin.freq(30000)
                PWM_pin.duty_u16(GeigerDuty(0))
                sleep(0.1)
                uart.write(b'radio tx ' + binascii.hexlify('ack') + b' 0\r\n')
                uart.readline()
                uart.readline()
                uart.readline()
                print("stopped")
                
            elif 'str' in message: #start
                uart.write(b'radio rxstop\r\n')
                uart.readline()
                tim.init(period=1, callback=Counter)
                sleep(0.1)
                uart.write(b'radio tx ' + binascii.hexlify('ack') + b' 0\r\n')
                uart.readline()
                uart.readline()
                uart.readline()
                print("start")
                
            elif 'mot' in message:
                parachute = 1
                print("motor on")
                sleep(0.1)
                uart.write(b'radio tx ' + binascii.hexlify('ack') + b' 0\r\n')
                uart.readline()
                uart.readline()
                uart.readline()
                
            elif 'ping' in message:
                print("PONG")
                sleep(0.1)
                uart.write(b'radio tx ' + binascii.hexlify('pong') + b' 0\r\n')
                print(uart.readline())
                print(uart.readline())
                print(uart.readline())
            
            elif 'gei' in message:
                geigerData = message.split(",")
                
                if "def" not in geigerData[1]:
                    PWM_pin.duty_u16(GeigerDuty(int(geigerData[1])))
                    dutyCycle = geigerData[1]
                    print("duty done")
                
                if "def" not in geigerData[2]:
                    print(int(geigerData[2][:-1]))
                    PWM_pin.freq((int(geigerData[2][:-1])))
                    print("geiger freq done")
                sleep(0.1)
                uart.write(b'radio tx ' + binascii.hexlify('ack') + b' 0\r\n')
                print(uart.readline())
                print(uart.readline())
                print(uart.readline())
