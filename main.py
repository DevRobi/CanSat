from machine import Pin, I2C, PWM, Timer, UART, SPI, ADC, WDT
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
fly_start_time = 0
PULL_TIME = 8500
HOLD_TIME = 35000
WAIT_TIME = 40000

#geiger--------------------------------------------------
PWM_pin = PWM(Pin(22))
INPUT_pin = Pin(20, Pin.IN)
ALERTON_led = Pin(7, Pin.OUT)
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

standardPressure = 1025.7 #in hPa
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

#gps---------------------------------------------
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

#motor--------------------------------------------------
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
    elif state == 1:
        if fly_start_time + WAIT_TIME <= cycleCount:
            return 2
    elif state == 2: # pull
        motor_on(1)
        motor_on_time = cycleCount
        print('Pull')
        return 3
    
    elif state == 3: # pulling on
        if motor_on_time + PULL_TIME <= cycleCount:
            motor_off()
            print('motor off')
            return 4
        
    elif state == 4: #hold on
        
        if motor_on_time + PULL_TIME + HOLD_TIME <= cycleCount:
            motor_on(-1)
            print('release')
            return 5
        
    elif state == 5: #release
        if motor_on_time + PULL_TIME + HOLD_TIME + PULL_TIME - 1800 <= cycleCount:
            motor_off()
            print('motor off')
            return 0
    
    #print(f'no change{state}')
    return state

#telemetry---------------------------------------------
class measurements_cansat:
    def __init__(self, timestamp, temperature, pressure, altitude, latitude, longitude, tic_count, geiger_vol, battery_vol, parachute_state):
        self.timestamp = timestamp  # int 4 bytes
        self.temp = temperature #float 4 bytes
        self.p = pressure  # float 4 bytes
        self.alt = altitude #float 4 bytes
        self.lat = latitude #float 4 bytes
        self.long = longitude #float 4 bytes
        self.tic_count = tic_count #int 2 bytes      
        self.geiger_vol = geiger_vol #int 2 bytes
        self.battery_vol = battery_vol #int 2 bytes
        self.parachute_state = parachute_state #int 1 bytes

tel = measurements_cansat(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

#radio------------------------------------------------
radioOk = 1
status = b''

# commandFreq = 868000000
# commandSf = 7

transmitFreq = b'866500000'
transmitSf = b'7'
transmitBw = b'250'

def Blink(t):
    global led
    led.value((led.value() - 1) * -1)

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
uart.write(b'radio set freq ' + transmitFreq + '\r\n')
print(uart.readline())
print("Radio set sf")
uart.write(b'radio set sf sf' + transmitSf + '\r\n')
print(uart.readline())
print("Radio set bw")
uart.write(b'radio set bw ' + transmitBw + '\r\n')
print(uart.readline())
print("Radio set power")
uart.write(b'radio set pa off\r\n')
print(uart.readline())
uart.write(b'radio set pwr 20\r\n')

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

def Measure():
    global ticCount, sampleIndex, ticsPerSecond, temperature, pressure, altitude, bme, sd
    global pressureLapseRate, standardPressure, latitude, longitude, tel, uart, timestamp, cycleCount
    global my_gps, presAltitude, batteryVol, batteryADC, geigerADC, geigerVol
    #BMP
    
    try:
        temperature = bme.read_temperature() / 100
        pressure = bme.read_pressure() / 25600
        presAltitude = (1 - math.pow((pressure / standardPressure), 0.190284)) * 145366.45 * 0.3048
    except:
        temperature = 0
        pressure = 0
        presAltitude = 0
    
    '''
    #geiger    
    print("geiger")
    print(ticcount)
    samples[sampleIndex] = ticCount
    print(samples)
    sampleIndex = (sampleIndex + 1) % numSamples
    
    sum = 0
    for sample in samples:
            sum += sample
    
    ticsPerSecond = int(round(sum / numSamples))
    
    ticcount = 0
    '''
    
    #extract lat, long
    if my_gps.fix_stat > 0:
        long = my_gps.longitude
        lat = my_gps.latitude
        altitude = my_gps.altitude
        longitude = long[0] + long[1] / 60
        latitude = lat[0] + lat[1] / 60
    else:
        longitude = 0.0
        latitude = 0.0
        altitude = 0.0
    
    #extract timestamp
    gps_timestamp = my_gps.timestamp
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
    timestamp = int(str_timestamp)
    
    #voltages
    batteryVol = int(batteryADC.read_u16() * 2 * 3300 / 65535)
    geigerVol = int(geigerADC.read_u16() * 148 * 3300 / 65535 / 1000)
    
    #processing
    tel.timestamp = timestamp
    tel.temp = temperature
    tel.p = pressure
    tel.alt = altitude
    tel.lat = latitude
    tel.long = longitude
    tel.tic_count = ticCount
    tel.geiger_vol = geigerVol
    tel.battery_vol = batteryVol
    tel.parachute_state = parachute
    
    if sdOk == 1:
        with open("/sd/telemetry.txt", "a") as f:
            f.write(str(tel.timestamp) + "," +
                    str(tel.temp) + "," +
                    str(tel.p) + "," +
                    str(tel.alt) + "," +                  
                    str(tel.lat) + "," +
                    str(tel.long) + "," +
                    str(tel.tic_count) + "," +
                    str(tel.geiger_vol) + "," +
                    str(tel.battery_vol) + "," +
                    str(tel.parachute_state) + "\n")
                    
    
#     uart.write(b'radio set freq ' + transmitFreq + b'\r\n')
#     uart.readline()
#     uart.write(b'radio set sf sf' + transmitSf + b'\r\n')
#     uart.readline()
    
    if radioOk == 1:
        led.value(1)
        radio_message = b'SPTM' + bytearray(struct.pack("lfffffhhhh", tel.timestamp, tel.temp, tel.p, tel.alt, tel.lat, tel.long, tel.tic_count, tel.geiger_vol, tel.battery_vol, tel.parachute_state))
        uart.write(b'radio rxstop\r\n')
        uart.readline()
        sleep(0.05)
        
        uart.write(b'radio tx ' + binascii.hexlify(radio_message) + b' 0\r\n')
        uart.readline()
        uart.readline()
        uart.readline()
        
        led.value(0)
    
    print(tel.timestamp, tel.temp, tel.p, tel.alt, tel.lat, tel.long, tel.tic_count, tel.geiger_vol, tel.battery_vol, tel.parachute_state)

    #cycleCount = 0
    
    uart.write(b'radio set freq ' + transmitFreq + b'\r\n')
    uart.readline()
    uart.write(b'radio set sf sf' + transmitSf + b'\r\n')
    uart.readline()
    
    uart.write(b'radio rx 0\r\n')
    uart.readline()
    
    
def Tic(t):
    global ticCount
    print("tic")
    ticCount += 1

def Counter(t):
    global cycleCount, measureFlag
    cycleCount += 1
    if cycleCount % 1000 == 0:
        measureFlag = 1

INPUT_pin.irq(handler=Tic, trigger=Pin.IRQ_RISING) #trigger

tim.init(period=1, callback=Counter) #timer that handles everything

wdt = WDT(timeout=2000)

while True:
    wdt.feed()
    
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
        for x in str(gpgga):
            my_gps.update(x)
    
    if uart.any():
        message = uart.readline().decode('utf-8').strip()
        print(message)
        if 'radio_rx' in message:
            message = binascii.unhexlify(message[9:]).decode('utf-8')
            print(message)
            uart.write(b'radio rxstop\r\n')
            uart.readline()
            
            if 'fre' in message:
                # fre,freq,baud,sf
                freqData = message.split(",")
                
                if "def" not in freqData[1]:
                    uart.write(b'radio set freq ' + freqData[1][1:] + '\r\n')
                    transmitFreq = freqData[1][1:]
                    print(uart.readline())
                    print("freqdone")
                
                if "def" not in freqData[2]:
                    uart.write(b'radio set bw ' + freqData[2] + '\r\n')
                    transmitBw = freqData[2]
                    print(uart.readline())
                    print("done")
                
                if "def" not in freqData[3]:
                    uart.write(b'radio set sf sf' + freqData[3][:-1] + '\r\n')
                    transmitSf = freqData[3][:-1]
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
                fly_start_time = cycleCount
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
                try:
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
                except:
                    uart.write(b'radio tx ' + binascii.hexlify('gei err') + b' 0\r\n')
                    print(uart.readline())
                    print(uart.readline())
                    print(uart.readline())
            
            elif 'prs' in message:
                if my_gps.fix_stat > 0:
                    standardPressure = pressure / (math.pow((1 - (altitude / (145366.45 * 0.3048))),(1/0.190284)))
                    print(standardPressure)
                    sleep(0.1)
                    
                    uart.write(b'radio tx ' + binascii.hexlify('ack') + b' 0\r\n')
                    uart.readline()
                    uart.readline()
                    uart.readline()
                else:
                    uart.write(b'radio tx ' + binascii.hexlify('no gps alt') + b' 0\r\n')
                    uart.readline()
                    uart.readline()
                    uart.readline()
