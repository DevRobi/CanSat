import socket
import urllib.request

def traccarUpload(ID,lat=0,lon=0,alt=0,battery=0,rssi=0,alarm=''):
	# Alarms: https://github.com/traccar/traccar/blob/master/src/main/java/org/traccar/model/Position.java#L85

		#if base has gps coordinates filter node coordinates that are too far away
		
		url = '10.0.0.153'
		port = 5055
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



traccarUpload(ord('A'),46.957738328486876, 17.450909368747798,140,3700/1000.0,-56)
# traccarUpload(ord('A'),lat,lon,alt,bat/1000.0,payload.rssi)
			