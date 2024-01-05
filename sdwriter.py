from machine import SPI, Pin
import sdcard
import uos

# Assign chip select (CS) pin (and start it high)
cs = Pin(9, Pin.OUT)

# Intialize SPI peripheral (start with 1 MHz)
spi = SPI(1,
            baudrate=1000000,
            polarity=0,
            phase=0,
            bits=8,
            firstbit=SPI.MSB,
            sck=Pin(14),
            mosi=Pin(15),
            miso=Pin(8))

# Initialize SD card
sd = sdcard.SDCard(spi, cs)

# Mount filesystem

vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")

class test:
    def __init__(self, first, second, third):
        self.first = first
        self.second = second
        self.third = third

message = test(1001, 200.002, 300.3)
# Create a file and write something to it
with open("/sd/test01.txt", "w") as file:
    file.write(str(message.first) + " " + str(message.second) + "\n")

with open("/sd/test01.txt", "a") as file:
    file.write("this is appended here")

# Open the file we just created and read from it
with open("/sd/test01.txt", "r") as file:
    data = file.read()
    print(data)