import time
import micropython
from inventor import Inventor2040W
from machine import Timer, Pin

board = Inventor2040W(init_encoders=False, init_leds=False)

m = board.motors[0]
while True:
    m.full_positive()
    time.sleep(1)
    m.stop()
    time.sleep(1)
