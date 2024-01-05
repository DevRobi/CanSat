from machine import Pin, UART
from time import sleep
from micropyGPS import MicropyGPS

gps = UART(1, 9600, tx=Pin(4), rx=Pin(5),timeout=100)
PMTK_SET_NMEA_OUTPUT_RMCGGA = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*"

def gps_send(msg):
    cs=0x00
    for c in msg :
        if c !='$' and c !='*':
            cs = cs ^ ord(c)
    gps_msg = msg + (chr( (cs>>4) + ord('0') )) + chr( (cs&0x0f) + ord('0') ) + '\r\n'
    gps.write(gps_msg)  
  

led = Pin(25, Pin.OUT)
ledgreen = Pin(15, Pin.OUT)
ledred = Pin(14, Pin.OUT)
gps_send(PMTK_SET_NMEA_OUTPUT_RMCGGA)
sleep(1)
while True:
    
    ###########################GPS
    gpgga = gps.readline()
    print(gpgga)
    '''
    my_gps = MicropyGPS(1)
    for x in str(gnrmc):
        my_gps.update(x)
    
    long = my_gps.longitude
    lat = my_gps.latitude
    altitude = my_gps.altitude
    longitude = long[0] + long[1] / 60
    latitude = lat[0] + lat[1] / 60
    print(longitude)
    print(latitude)
    print(altitude)
    gps_timestamp = my_gps.timestamp
    str_timestamp = ""
    
    for x in gps_timestamp: # tuple
        x = int(x)
        if x < 10:
            x = str("0{0}".format(x))
        else:
            x = str(x)
        str_timestamp += x
    timestamp = int(str_timestamp)
    print(timestamp)
            
    #########################GPS
    
    ledgreen.value(0)
       
    sleep(.5)
    ledgreen.value(1)
    sleep(.5)
    '''

