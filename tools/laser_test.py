import time
import micropython
from inventor import Inventor2040W
from machine import Timer, Pin

board = Inventor2040W(init_encoders=False, init_leds=False)

laser = Pin(Inventor2040W.ENCODER_A_PINS[0], Pin.OUT)
while True:
    laser.on()
    time.sleep(2)
    laser.off()
    time.sleep(1)
