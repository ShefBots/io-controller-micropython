import time
import micropython
from inventor import Inventor2040W, SERVO_1
from machine import Timer, Pin
from servo import Calibration

board = Inventor2040W(init_encoders=False, init_leds=False)

s = board.servos[SERVO_1]

cal = Calibration()
cal.apply_two_pairs(700, 2500, 0, 30)
s.calibration(cal)
#s.enable()
#time.sleep(2)

# Go to min
#while True:
#s.to_min()
#time.sleep(2)

s.to_max()
#time.sleep(2)

