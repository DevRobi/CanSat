import struct
class measurements_cansat:
    def __init__(self, temperature, altitude, pressure, latitude, longitude, tics_per_sec, geiger_vol, battery_vol):
        self.temp = temperature #float 4 bytes
        self.alt = altitude #float 4 bytes
        self.p = pressure #float 4 bytes
        self.lat = latitude #float 4 bytes
        self.long = longitude #float 4 bytes
        self.tics_per_sec = tics_per_sec #int 2 bytes
        self.geiger_vol = geiger_vol #int 2 bytes
        self.battery_vol = battery_vol #int 2 bytes
        
# create object
m = measurements_cansat(0, 0, 0, 0, 0, 0)
# struct pack data, send it
radio_message = bytearray(struct.pack("fffffh", m.temp, m.alt, m.p, m.lat, m.long, m.tics_per_sec))
# data sent #########################
#receiver side creates object
telemetry = measurements_cansat(0,0,0,0,0,0)
# change object's values by struct.unpack sent data
[telemetry.temp, telemetry.alt, telemetry.p, telemetry.lat, telemetry.long, telemetry.tics_per_sec] = struct.unpack("fffffh",radio_message)
# print out data
print(telemetry.temp, telemetry.alt, telemetry.p, telemetry.lat, telemetry.long, telemetry.tics_per_sec)

with open("log.txt", "a") as f:
    # Write new data to the file
    f.write(str((telemetry.temp, telemetry.alt, telemetry.p, telemetry.lat, telemetry.long, telemetry.tics_per_sec)))
    f.write("\n")
    f.close()