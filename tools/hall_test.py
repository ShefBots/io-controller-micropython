import time
import micropython
from inventor import Inventor2040W
from machine import Timer, Pin

board = Inventor2040W(init_encoders=False, init_leds=False)

hall = Pin(Inventor2040W.ENCODER_A_PINS[1], Pin.IN, Pin.PULL_UP)
while True:
    first = hall.value()
    time.sleep(0.01)
    second = hall.value()
    if first and not second:
        print('Hall pressed!')
    elif not first and second:
        print('Hall released!')