from machine import Pin, UART
from time import sleep
from micropyGPS import MicropyGPS
NPNm=Pin(2, Pin.OUT)
NPNp=Pin(13, Pin.OUT)
PNPm=Pin(12, Pin.OUT)
PNPp=Pin(3, Pin.OUT)
PPS=Pin(21, Pin.IN)
gps = UART(1, 9600, tx=Pin(4), rx=Pin(5),timeout=100)
radio = UART(0, 115200, tx=Pin(16), rx=Pin(17),timeout=1000)
PMTK_SET_NMEA_OUTPUT_RMCGGA = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*"

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

def gps_send(msg):
    cs=0x00
    for c in msg :
        if c !='$' and c !='*':
            cs = cs ^ ord(c)
    gps_msg = msg + (chr( (cs>>4) + ord('0') )) + chr( (cs&0x0f) + ord('0') ) + '\r\n'
    gps.write(gps_msg)  

motor_off()
#motor_on(-1)
led = Pin(25, Pin.OUT)
ledgreen = Pin(15, Pin.OUT)
ledred = Pin(14, Pin.OUT)
gps_send(PMTK_SET_NMEA_OUTPUT_RMCGGA)
sleep(1)
radio.write(b'sys reset\r\n')
print(radio.readline())
print(radio.readline())
print(radio.readline())
print(radio.readline())
radio.write(b'radio set freq 868000000\r\n')
print(radio.readline())
radio.write(b'radio set sf sf8\r\n')
print(radio.readline())
while True:
    
    ###########################GPS
    print("GPS data")
    gnrmc = gps.readline()
    gpgga = gps.readline()
    print(gnrmc)
    print(gpgga)
    my_gps = MicropyGPS(1)
    for x in str(gpgga):
        my_gps.update(x)
    
    long = my_gps.longitude
    lat = my_gps.latitude
    float_long = long[0] + long[1] / 60
    float_lat = lat[0] + lat[1] / 60
    print(float_long)
    print(float_lat)
    timestamp = my_gps.timestamp
    str_timestamp = ""
    
    for x in timestamp: # tuple
        x = int(x)
        if x < 10:
            x = str("0{0}".format(x))
        else:
            x = str(x)
        str_timestamp += x
    int_timestamp = int(str_timestamp)
    print(timestamp)
    print(int_timestamp)
            
    #########################GPS
    
    ledgreen.value(0)
    '''radio.write(b'radio tx 0123456 1\r\n')
    print(radio.readline())
    print(radio.readline())
    print(radio.readline())'''    
    sleep(.5)
    ledgreen.value(1)
    sleep(.5)
    
