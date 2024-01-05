from machine import Pin, I2C
from time import sleep
import BME280
import math

standardPressure = 1013.25 #in hPa
pressureLapseRate = 1/9.14 #hPa/m

sda = Pin(10)
scl = Pin(11)
i2c = I2C(1, sda=sda, scl=scl, freq=400000)

while True:
    bme = BME280.BME280(i2c=i2c)
    
    temp = bme.temperature
    pres = bme.pressure
    hum = bme.humidity
    
    presFloat = float(pres[:-3])
    
    altitude = (1 - math.pow((presFloat / standardPressure), 0.190284)) * 145366.45 * 0.3048
    
    print('Temperature: ', temp, ' Pressure: ', pres, ' Altitude: ', altitude, ' Humidity ', hum)
    
    sleep(2)