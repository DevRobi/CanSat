from machine import Pin, PWM, Timer

led_pin = Pin(25, Pin.OUT)
PWM_pin = PWM(Pin(20))
INPUT_pin = Pin(18, Pin.IN)

dutyCycle = 0
frequency = 30000

ticCount = 0
numSamples = 10
samples = [0] * numSamples
sampleIndex = 0
ticsPerSecond = 0

tim = Timer()

PWM_pin.freq(frequency)
PWM_pin.duty_u16(100-dutyCycle)
led_pin.value(dutyCycle)


def CalcTics(t):
    global ticCount, sampleIndex, ticsPerSecond
    
    samples[sampleIndex] = ticCount
    sampleIndex = (sampleIndex + 1) % numSamples
    
    sum = 0
    for sample in samples:
            sum += sample
    
    ticsPerSecond = int(round(sum / numSamples))
    
    print("Tics per second = ", ticsPerSecond)
    
    ticCount = 0

def Tic():
    ticCount += 1

INPUT_pin.irq(handler=Tic, trigger=Pin.IRQ_RISING)

tim.init(freq=1, mode=Timer.PERIODIC, callback=CalcTics)