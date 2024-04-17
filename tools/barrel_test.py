import time
import micropython
from inventor import Inventor2040W, A1
from machine import Timer, Pin

board = Inventor2040W(init_encoders=False, init_leds=False)

barrel = Pin(A1, Pin.IN)
while True:
    first = barrel.value()
    time.sleep(0.01)
    second = barrel.value()
    if first and not second:
        print('barrel present!')
    elif not first and second:
        print('No barrel!')