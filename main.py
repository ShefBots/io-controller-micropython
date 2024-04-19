import time
import micropython
from inventor import Inventor2040W, GP0, GP1, GP2, A0, A1, A2
from comms.serial import make_command, USBSerialComms, SSHORT, UBYTE, USHORT
from sensors.vl53l4cd import VL53L4CD
from machine import Timer, Pin
from plasma import WS2812
from servo import Servo, Calibration

# Constants
BOARD_ID = 0x01     # The ID this board will report back on USB serial
NUM_LEDS = 7        # The number of addressable LEDs that are controllable
LED_BRIGHTNESS = 64 # The max brightness the LEDs will go to

# The time of flight sensor indices
RIGHT_TOF = 0
FRONT_TOF = 1
LEFT_TOF = 2
BACK_TOF = 3

# Gripper states
GRIPPER_CLOSED = 0
GRIPPER_OPEN = 1
GRIPPER_UNKNOWN = 2
GRIPPER_CLOSING = 3
GRIPPER_OPENING = 4

# Gripper constants
GRIPPER_TIMESTEP = 20        # The time between each servo update
GRIPPER_DURATION_MS = 1000   # The duration of a gripper open/close
GRIPPER_TIMEOUT_MS = 500     # How long to wait after a gripper move before turning the servos off
GRIPPER_OPEN_ANGLE = 0       # The servo angle for open
GRIPPER_CLOSED_ANGLE = 45    # The servo angle for closed

# Turret states
TURRET_LOADING = 1
TURRET_LOADING_HOLD = 2
TURRET_FIRING = 3
TURRET_FIRING_HOLD = 4
TURRET_RETURN = 5
TURRET_RETURN_HOLD = 6

# Turret constants
TURRET_TIMESTEP = 50
TURRET_LOADING_HOLD_MS = 500
TURRET_FIRING_HOLD_MS = 1000
TURRET_RETURN_HOLD_MS = 1000

# USB Serial command definitions
COM_IDENTIFY_RECV = make_command('I')
COM_IDENTIFY_ACK = make_command('I', UBYTE)

COM_POKE_RECV = make_command('P')   # No Poke ACK

COM_READ_TOF_RECV = make_command('T', UBYTE)
COM_READ_TOF_ACK = make_command('T', SSHORT)

COM_SET_LED_RECV = make_command('L', UBYTE * 4)     # No Set LED ACK

COM_SET_GRIPPER_RECV = make_command('G', UBYTE)     # No Set Gripper ACK
COM_READ_GRIPPER_RECV = make_command('g')
COM_READ_GRIPPER_ACK = make_command('g', UBYTE)

COM_READ_BARREL_RECV = make_command('B')
COM_READ_BARREL_ACK = make_command('B', UBYTE)

COM_POWER_TURRET_RECV = make_command('W', UBYTE)        # No Turret ACK
COM_SET_TURRET_TILT_RECV = make_command('E', USHORT)    # No Tilt ACK
COM_SET_TURRET_SPEED_RECV = make_command('S', USHORT)   # No Speed ACK
COM_FIRE_TURRET_RECV = make_command('F', USHORT)        # No Fire ACK


# Initialise the Inventor 2040 and systems
board = Inventor2040W(init_encoders=False, init_servos=False, init_leds=False)
comms = USBSerialComms()
leds = WS2812(NUM_LEDS, 0, 2, board.SERVO_6_PIN)
leds.start()

# Show the board has powered up
for i in range(NUM_LEDS):
    leds.set_rgb(i, LED_BRIGHTNESS, LED_BRIGHTNESS, 0)
time.sleep(0.5)

gripper_servo_l = Servo(Inventor2040W.SERVO_1_PIN)
gripper_servo_r = Servo(Inventor2040W.SERVO_2_PIN)
turret_tilt_servo = Servo(Inventor2040W.SERVO_3_PIN)
turret_fire_servo = Servo(Inventor2040W.SERVO_4_PIN)
turret_brushless = Servo(GP0)
turret_motor = board.motors[0]
turret_hall = Pin(Inventor2040W.ENCODER_A_PINS[1], Pin.IN, Pin.PULL_UP)
turret_laser = Pin(Inventor2040W.ENCODER_A_PINS[0], Pin.OUT)

# Calibrate the gripper servos
l_cal = Calibration()
l_cal.apply_two_pairs(1500, 990, 0, 45)
gripper_servo_l.calibration(l_cal)

r_cal = Calibration()
r_cal.apply_two_pairs(1550, 2060, 0, 45)
gripper_servo_r.calibration(r_cal)

# Calibrate the turret servos
t_cal = Calibration()
t_cal.apply_two_pairs(700, 2500, 0, 30)
turret_tilt_servo.calibration(t_cal)

f_cal = Calibration()
f_cal.apply_two_pairs(1500, 2250, 0, 1)
turret_fire_servo.calibration(f_cal)

b_cal = Calibration()
b_cal.apply_two_pairs(500, 2500, 0, 1)
turret_brushless.calibration(b_cal)


tof_sensors = []

# Set up the ToF shutdown pins
xshut_pins = [Pin(GP1, Pin.OUT),
              Pin(GP2, Pin.OUT),
              Pin(A0, Pin.OUT),
              Pin(A1, Pin.OUT)]

gripper_state = GRIPPER_UNKNOWN
gripper_timer = Timer(-1)

turret_powered = False
turret_timer = Timer(-1)
turret_timer_running = False

barrel_sensor = Pin(A2, Pin.IN, Pin.PULL_UP)


# Shut all the ToF sensors down
for xshut in xshut_pins:
    xshut.low()

# Initialise each ToF with a unique address based on its shutdown pin
for i, xshut in enumerate(xshut_pins):
    xshut.high()
    try:
        vl53 = VL53L4CD(board.i2c)
        vl53.set_address(0x41 + 1 + i)

        #vl53.inter_measurement = 0
        vl53.timing_budget = 1000 // 10
    except OSError as e:
        vl53 = None
    tof_sensors.append(vl53)

# Raise a warning if some of the ToF sensors are missing
if tof_sensors.count(None) > 0:
    for led in range(NUM_LEDS):
        if led < len(xshut_pins):
            if tof_sensors[led] is None:
                leds.set_rgb(NUM_LEDS - led - 1, LED_BRIGHTNESS, 0, LED_BRIGHTNESS)
            else:
                leds.set_rgb(NUM_LEDS - led - 1, 0, LED_BRIGHTNESS, 0)
        else:
            leds.set_rgb(NUM_LEDS - led - 1, 0, 0, 0)
    while not board.switch_pressed():
        pass

# List storing the distances from the ToFs
last_distance = [0] * len(tof_sensors)


# Show the board has passed initialisation
for i in range(NUM_LEDS):
    leds.set_rgb(i, LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS)


# Comms callback functions
def comms_connected():
    # Update all the LEDs
    for i in range(NUM_LEDS):
        leds.set_rgb(i, 0, LED_BRIGHTNESS, 0)
    for vl53 in tof_sensors:
        if vl53 is not None:
            vl53.start_ranging()

def comms_disconnected():
    # Update all the LEDs
    for i in range(NUM_LEDS):
        leds.set_rgb(i, 0, 0, LED_BRIGHTNESS)
    for vl53 in tof_sensors:
        if vl53 is not None:
            vl53.stop_ranging()

    gripper_timer.deinit()
    gripper_servo_l.disable()
    gripper_servo_r.disable()

    turret_timer.deinit()
    turret_timer_running = False
    turret_tilt_servo.disable()
    turret_fire_servo.disable()
    turret_brushless.disable()
    turret_motor.disable()
    turret_laser.off()    

# Command callback functions
def identify_ack():
    comms.send(COM_IDENTIFY_ACK, "B", BOARD_ID)

def read_tof_ack(data):
    if data >= 0 and data < len(tof_sensors):
        comms.send(COM_READ_TOF_ACK, "H", int(last_distance[data] * 10))

def set_led(data):
    led, r, g, b = data
    if led < NUM_LEDS:
        leds.set_rgb(led, (r * LED_BRIGHTNESS) // 255, (g * LED_BRIGHTNESS) // 255, (b * LED_BRIGHTNESS) // 255)

def set_gripper(state):
    global gripper_timer
    global gripper_state
    if state == GRIPPER_OPEN:
        end_value = GRIPPER_OPEN_ANGLE
        target_state = GRIPPER_OPEN
        gripper_state = GRIPPER_OPENING

    elif state == GRIPPER_CLOSED:
        end_value = GRIPPER_CLOSED_ANGLE
        target_state = GRIPPER_CLOSED
        gripper_state = GRIPPER_CLOSING

    else:
        gripper_servo_l.disable()
        gripper_servo_r.disable()
        gripper_state = GRIPPER_UNKNOWN
        return
    
    gripper_timer.deinit()

    gripper_servo_l.enable()
    gripper_servo_r.enable()

    time_elapsed = 0
    start_value_l = gripper_servo_l.value()
    start_value_r = gripper_servo_r.value()

    def gripper_update(timer):
        nonlocal time_elapsed
        global gripper_state
        time_elapsed += GRIPPER_TIMESTEP
        if time_elapsed >= GRIPPER_DURATION_MS + GRIPPER_TIMEOUT_MS:
            timer.deinit()  # Stop the timer when duration is reached
            gripper_servo_l.disable()
            gripper_servo_r.disable()

        elif time_elapsed >= GRIPPER_DURATION_MS:
            gripper_servo_l.value(end_value)
            gripper_servo_r.value(end_value)
            gripper_state = target_state

        else:
            gripper_servo_l.to_percent(time_elapsed, 0, GRIPPER_DURATION_MS, start_value_l, end_value)
            gripper_servo_r.to_percent(time_elapsed, 0, GRIPPER_DURATION_MS, start_value_r, end_value)

    gripper_timer.init(period=GRIPPER_TIMESTEP, mode=Timer.PERIODIC, callback=gripper_update)

def read_gripper_ack():
    comms.send(COM_READ_GRIPPER_ACK, "B", gripper_state)

def power_turret(state):
    global turret_powered
    if state == 1:
        turret_tilt_servo.to_min()
        turret_fire_servo.to_min()
        turret_brushless.enable()
        turret_motor.stop()
        turret_laser.on()
    else:
        turret_timer.deinit()
        turret_timer_running = False
        turret_tilt_servo.disable()
        turret_fire_servo.disable()
        turret_brushless.disable()
        turret_motor.disable()
        turret_laser.off()

    turret_powered = state
        
def turret_tilt(angle):
    global turret_powered
    if turret_powered:
        turret_tilt_servo.value(angle / 1000)

def turret_speed(speed):
    global turret_powered
    if turret_powered:
        turret_tilt_servo.value(speed / 1000)

def fire_turret():
    global turret_timer
    global turret_powered
    global turret_timer_running
    
    if not turret_powered:
        return
    
    if turret_timer_running:
        return

    turret_timer.deinit()   # Should be unnecessary

    time_elapsed = 0
    last_hall = turret_hall.value()
    
    # Is the magazine already aligned?
    if last_hall == 1:
        turret_motor.stop()
        turret_state = TURRET_FIRING
    else:
        turret_motor.full_positive()
        turret_state = TURRET_LOADING

    def turret_update(timer):
        nonlocal time_elapsed
        nonlocal last_hall
        nonlocal turret_state
        
        current_hall = turret_hall.value()
        
        if turret_state == TURRET_LOADING:
            #print("Loading")
            if current_hall == 1 and last_hall == 0:
                turret_motor.stop()
                time_elapsed = 0
                turret_state = TURRET_LOADING_HOLD

        elif turret_state == TURRET_LOADING_HOLD:
            time_elapsed += TURRET_TIMESTEP
            #print(f"Loading - Hold {time_elapsed}")
            if time_elapsed >= TURRET_LOADING_HOLD_MS:
                turret_state = TURRET_FIRING
                
        elif turret_state == TURRET_FIRING:
            #print("Firing")
            turret_fire_servo.to_max()
            time_elapsed = 0
            turret_state = TURRET_FIRING_HOLD
            
        elif turret_state == TURRET_FIRING_HOLD:
            time_elapsed += TURRET_TIMESTEP
            #print(f"Firing - Hold {time_elapsed}")
            if time_elapsed >= TURRET_FIRING_HOLD_MS: #ms
                turret_state = TURRET_RETURN
                
        elif turret_state == TURRET_RETURN:
            #print("Return")
            turret_fire_servo.to_min()
            time_elapsed = 0
            turret_state = TURRET_RETURN_HOLD
        
        elif turret_state == TURRET_RETURN_HOLD:
            time_elapsed += TURRET_TIMESTEP
            #print(f"Return - Hold {time_elapsed}")
            if time_elapsed >= TURRET_RETURN_HOLD_MS:
                #print("Done")
                timer.deinit()  # Stop the timer when duration is reached
                turret_timer_running = False

    turret_timer.init(period=TURRET_TIMESTEP, mode=Timer.PERIODIC, callback=turret_update)
    turret_timer_running = True

def read_barrel_ack():
    comms.send(COM_READ_BARREL_ACK, "B", not barrel_sensor.value())


# Command setup
comms.set_comms_established_callback(comms_connected)
comms.set_no_comms_timeout_callback(comms_disconnected)

comms.assign(COM_POKE_RECV)
comms.assign(COM_IDENTIFY_RECV, identify_ack)
comms.assign(COM_READ_TOF_RECV, read_tof_ack)
comms.assign(COM_SET_LED_RECV, set_led)
comms.assign(COM_SET_GRIPPER_RECV, set_gripper)
comms.assign(COM_READ_GRIPPER_RECV, read_gripper_ack)

comms.assign(COM_POWER_TURRET_RECV, power_turret)
comms.assign(COM_SET_TURRET_TILT_RECV, turret_tilt)
comms.assign(COM_SET_TURRET_SPEED_RECV, turret_speed)
comms.assign(COM_FIRE_TURRET_RECV, fire_turret)

comms.assign(COM_READ_BARREL_RECV, read_barrel_ack)

# Credit to DrFootleg
time.sleep(5)               # Sleep to allow time to stop the program on power up to get to repl
micropython.kbd_intr(-1)    # Disable reacting to escape characters

# Animate the gripper to verify it's working
set_gripper(GRIPPER_CLOSED)
time.sleep_ms(GRIPPER_DURATION_MS)
set_gripper(GRIPPER_OPEN)
time.sleep_ms(GRIPPER_DURATION_MS)
set_gripper(GRIPPER_CLOSED)
time.sleep_ms(GRIPPER_DURATION_MS)

# Show the board is now functioning
comms_disconnected()

# TEMP
#power_turret(1)
#fire_turret()

# TEMP
#for vl53 in tof_sensors:
#    vl53.start_ranging()

# Variables for the main loop
tof_read_fails = [0] * len(tof_sensors)
tof_read_invalid = [0] * len(tof_sensors)

# The main program loop
while not board.switch_pressed():
    comms.check_receive()
    for i, vl53 in enumerate(tof_sensors):
        if vl53 is not None:
            try:
                if vl53.data_ready:
                    tof_read_fails[i] = 0
                    vl53.clear_interrupt()
                    status = vl53.range_status()
                    rate = vl53.signal_rate()
                    distance = vl53.distance
                    if status != 4 and distance > 0:# and rate >= 500:
                        last_distance[i] = distance
                        #print("[{}] Distance: {} cm, Status {}, Sigma {}, Sig Rate {}".format(i, distance, status, vl53.sigma(), rate))
                        tof_read_invalid[i] = 0
                    else:
                        tof_read_invalid[i] += 1
                        if tof_read_invalid[i] > 5:
                            last_distance[i] = -69
                            #print("ToF Not Detecting")
            except OSError:
                tof_read_fails[i] += 1
                #print("ToF Error")
                if tof_read_fails[i] > 2:
                    last_distance[i] = -99
                    #print("ToF Failed")

# Turn off the LEDs to show the program has ended
for i in range(NUM_LEDS):
    leds.set_rgb(i, 0, 0, 0)

# Disable the gripper servos
gripper_servo_l.disable()
gripper_servo_r.disable()
