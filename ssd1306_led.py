import machine, framebuf
import UART, Pin
i2c = machine.I2C(0,sda=machine.Pin(4), scl=machine.Pin(5))
x = i2c.scan()
print(x)
from ssd1306 import SSD1306_I2C
oled = SSD1306_I2C(128, 32, i2c)
while True:
    if uart1.any()>0:
        x = uart1.read().decode() 
oled.text(x, 30, 30)
oled.show()