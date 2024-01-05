import struct
import serial
import time
import binascii
import threading
import datetime
import math

satOn = True

import socket
import urllib.request

def traccarUpload(ID,lat=0,lon=0,alt=0,battery=0,rssi=0,alarm=''):
	# Alarms: https://github.com/traccar/traccar/blob/master/src/main/java/org/traccar/model/Position.java#L85

		#if base has gps coordinates filter node coordinates that are too far away
		
		url = '10.0.0.1'
		port = 8082
		s = socket.socket()
		s.settimeout(1)
		try:
			s.connect((url, port))
			if alarm != '':
				s.send(bytes('POST /?id={}&alarm={} HTTP/1.1\r\nHost: {}:{}\r\n\r\n'\
							.format(ID,  alarm, url, port), 'utf8'))
				print(f"Alarm from [{chr(ID)}]: {alarm}")
			else:
				s.send(bytes('POST /?id={}&lat={}&lon={}&altitude={}&battery={}&rssi={}&alarm={} HTTP/1.1\r\nHost: {}:{}\r\n\r\n'\
							.format(ID, lat, lon, alt, battery, rssi, alarm, url, port), 'utf8'))
			ret = s.recv(50).decode()
			if "200 OK" not in ret:
				print(ret) # Should be b'HTTP/1.1 200 OK\r\ncontent-length: 0\r\n\r\n'
		except OSError as e:
			print("Traccar upload error: ",e)


		s.close()

def calc_distance(lat1,lon1,lat2,lon2):
	R=6371.0
	r_lat1 = lat1 * (math.pi/180.0)
	r_lat2 = lat2 * (math.pi/180.0)
	dlon = (lon2 - lon1) * (math.pi/180.0)
	dlat = (lat2 - lat1) * (math.pi/180.0)

	a = math.sin(dlat / 2)**2 + math.cos(r_lat1)* math.cos(r_lat2) * math.sin(dlon/2)**2
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
	d = R*c
	return d * 1000.0


class measurements_cansat:
    def __init__(self, timestamp, temperature, pressure, altitude, latitude, longitude, tics_per_sec, geiger_vol, battery_vol, parachute_state, rssi):
        self.timestamp = timestamp  # int 4 bytes
        self.temp = temperature #float 4 bytes
        self.p = pressure  # float 4 bytes
        self.alt = altitude #float 4 bytes
        self.lat = latitude #float 4 bytes
        self.long = longitude #float 4 bytes
        self.tics_per_sec = tics_per_sec #int 2 bytes      
        self.geiger_vol = geiger_vol #int 2 bytes
        self.battery_vol = battery_vol #int 2 bytes
        self.parachute_state = parachute_state #int 1 byte
        self.rssi = rssi #rssi

consoleBuffer = []

def consoleInput(myBuffer):
    while True:
        myBuffer.append(input())

threading.Thread(target=consoleInput, args=(consoleBuffer,), daemon=True).start() # start the thread
        
# initialize UART
uart = serial.Serial('COM6', 115200, timeout=100)
uart.write(b'sys reset\r\n')
uart.readline()
uart.readline()
uart.readline()
uart.readline()
# set frequency and spreading factor
uart.write(b'radio set freq 866500000\r\n')
print(uart.readline())
uart.write(b'radio set sf sf7\r\n')
print(uart.readline())
uart.write(b'radio set bw 250\r\n')
print(uart.readline())
uart.write(b'radio set pa off\r\n')
print(uart.readline())
uart.write(b'radio set pwr 20\r\n')
print(uart.readline())
uart.write(b'radio get pwr\r\n')
print(uart.readline())





# start listening for incoming messages
uart.write(b'radio rx 0\r\n')
uart.readline()


while True:
    # check if a message has been received
    if satOn:
        if uart.in_waiting:
            # read the incoming message
            
            message = uart.readline().decode('utf-8').strip()
            
            if 'radio_rx' in message:
                message = bytearray(binascii.unhexlify(message[9:]))
                if b'pong' in message:
                    print("pong")
                elif b'booted' in message:
                    print(message.decode('utf-8'))
                elif b'ack' in message:
                    print("ack") 
                elif b'no gps alt' in message:
                    print("no gps altitude")
                elif b'SPTM' in message:
                    message = message[4:]
                    telemetry = measurements_cansat(0,0,0,0,0,0,0,0,0,0,0)
                    # change object's values by struct.unpack sent data
                    try:
                        [telemetry.timestamp, telemetry.temp, telemetry.p, telemetry.alt, telemetry.lat, 
                        telemetry.long, telemetry.tics_per_sec,
                        telemetry.geiger_vol, telemetry.battery_vol, telemetry.parachute_state] = struct.unpack("lfffffhhhh",message)
                    except: 
                        print("telemetry error")
                        print(message)

                    now = datetime.datetime.now()

                    # Format time into HHMMSS string
                    #telemetry.timestamp = now.strftime("%H%M%S")
                    uart.write(b'radio get pktrssi\r\n')
                    telemetry.rssi = uart.readline().decode('utf-8').strip()
                
                    # print out data
                    print(telemetry.timestamp, round(telemetry.temp,1), round(telemetry.p, 1), int(telemetry.alt), round(telemetry.lat, 4), round(telemetry.long, 4), telemetry.tics_per_sec,
                        telemetry.geiger_vol, telemetry.battery_vol, telemetry.parachute_state, telemetry.rssi)
                    with open("log.txt", "a") as f:
                        f.write(str((telemetry.timestamp, 
                                    round(telemetry.temp, 1),
                                    round(telemetry.p, 1),
                                    int(telemetry.alt),
                                    round(telemetry.lat, 4), 
                                    round(telemetry.long, 4), 
                                    telemetry.tics_per_sec,   
                                    telemetry.geiger_vol, 
                                    telemetry.battery_vol,
                                    telemetry.parachute_state,
                                    telemetry.rssi)))
                        f.write("\n")
                        #f.close()
                    traccarUpload(ord('A'),telemetry.lat,telemetry.long,telemetry.alt,telemetry.battery_vol/1000.0,telemetry.rssi)
                    print(calc_distance(46.95847, 17.4476, telemetry.lat, telemetry.long))
                else:
                    print("unknown transmission")
                    print(message)

                if len(consoleBuffer) > 0:
                    radio_message = repr(consoleBuffer.pop(0))

                    uart.write(b'radio rxstop\r\n')
                    uart.readline()

                    uart.write(b'radio tx ' + binascii.hexlify(str.encode(radio_message)) + b' 0\r\n')
                    uart.readline()
                    uart.readline()
                    uart.readline()
                    if 'fre' in radio_message:
                        # fre,freq,baud,sf
                        freqData = radio_message.split(",")
                        
                        if freqData[1] != "def":
                            uart.write(b'radio set freq ' + str.encode(freqData[1][1:]) + b'\r\n')
                            uart.readline()
                            uart.write(b'radio get freq\r\n')
                            print(uart.readline())
                        
                        if freqData[2] != "def":
                            uart.write(b'radio set bw ' + str.encode(freqData[2]) + b'\r\n')
                            uart.readline()
                            uart.write(b'radio get bw\r\n')
                            print(uart.readline())
                        
                        if freqData[3] != "def":
                            uart.write(b'radio set sf sf' + str.encode(freqData[3][:-1]) + b'\r\n')
                            uart.readline()
                            uart.write(b'radio get sf\r\n')
                            print(uart.readline())
                        
                        uart.write(b'radio rx 0\r\n')
                        uart.readline()

                    elif 'stp' in radio_message: #stop
                        uart.write(b'radio rx 0\r\n')
                        uart.readline()
                        print("stopped")
                    elif 'str' in radio_message: #start
                        uart.write(b'radio rx 0\r\n')
                        uart.readline()
                        satOn = True
                        print("started")
                    elif 'mot' in radio_message:
                        uart.write(b'radio rx 0\r\n')
                        uart.readline()
                        print("motor triggered")
                    elif 'ping' in radio_message:
                        uart.write(b'radio rx 0\r\n')
                        uart.readline()
                        print("ping")
                    elif 'gei' in radio_message:
                        uart.write(b'radio rx 0\r\n')
                        uart.readline()
                        print("geiger")
                    else:
                        uart.write(b'radio rx 0\r\n')
                        uart.readline()                   
    else:
        if len(consoleBuffer) > 0:
            radio_message = repr(consoleBuffer.pop(0))

            uart.write(b'radio rxstop\r\n')
            uart.readline()

            uart.write(b'radio tx ' + binascii.hexlify(str.encode(radio_message)) + b' 0\r\n')
            uart.readline()
            uart.readline()
            uart.readline()
            if 'fre' in radio_message:
                # fre,freq,baud,sf
                freqData = radio_message.split(",")
                
                if freqData[1] != "def":
                    uart.write(b'radio set freq ' + str.encode(freqData[1][1:]) + b'\r\n')
                    uart.readline()
                    uart.write(b'radio get freq\r\n')
                    print(uart.readline())
                
                if freqData[2] != "def":
                    uart.write(b'radio set bw ' + str.encode(freqData[2]) + b'\r\n')
                    uart.readline()
                    uart.write(b'radio get bw\r\n')
                    print(uart.readline())
                
                if freqData[3] != "def":
                    uart.write(b'radio set sf sf' + str.encode(freqData[3][:-1]) + b'\r\n')
                    uart.readline()
                    uart.write(b'radio get sf\r\n')
                    print(uart.readline())
                
                uart.write(b'radio rx 0\r\n')
                uart.readline()

            elif 'stp' in radio_message: #stop
                uart.write(b'radio rx 0\r\n')
                uart.readline()
                print("stopped")
            elif 'str' in radio_message: #start
                uart.write(b'radio rx 0\r\n')
                uart.readline()
                satOn = True
                print("started")
            elif 'mot' in radio_message:
                uart.write(b'radio rx 0\r\n')
                uart.readline()
                print("motor triggered")
            elif 'ping' in radio_message:
                uart.write(b'radio rx 0\r\n')
                uart.readline()
                print("ping")
            elif 'gei' in radio_message:
                uart.write(b'radio rx 0\r\n')
                uart.readline()
                print("geiger")
            else:
                uart.write(b'radio rx 0\r\n')
                uart.readline()