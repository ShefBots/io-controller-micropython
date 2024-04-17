import time
import micropython
from inventor import Inventor2040W, SERVO_1
from machine import Timer, Pin
from servo import Calibration

board = Inventor2040W(init_encoders=False, init_leds=False)

hall = Pin(Inventor2040W.ENCODER_A_PINS[1], Pin.IN, Pin.PULL_UP)
laser = Pin(Inventor2040W.ENCODER_A_PINS[0], Pin.OUT)
m = board.motors[0]
s = board.servos[SERVO_1]

cal = Calibration()
cal.apply_two_pairs(1500, 2250, 0, 1)
s.calibration(cal)
s.to_min()
time.sleep(1)

last_hall = hall.value()
while True:
    current_hall = hall.value()
    if current_hall == 1 and last_hall == 0:
        m.stop()
        time.sleep(0.5) #was 0.1
        s.to_max()
        time.sleep(1) # was 0.6
        s.to_min()
        time.sleep(1) # was 0.6
  
    last_hall = current_hall
    m.full_positive()
    
    
# Turret elevation is 30 degrees
    
