import serial

## listen for incoming radio data
ser = serial.Serial('COM6', 115200, timeout=0.050)

while 1:
    while ser.in_waiting:
        data_in = ser.readline()
        print(data_in)