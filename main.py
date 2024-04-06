import time
import micropython
from inventor import Inventor2040W, NUM_LEDS
from comms.serial import make_command, USBSerialComms, UBYTE, USHORT
from sensors.vl53l4cd import VL53L4CD
from machine import Timer, Pin

BOARD_ID = 0x01
RIGHT_TOF = 0
FRONT_TOF = 1
LEFT_TOF = 2

COM_IDENTIFY_RECV = make_command('I')
COM_IDENTIFY_ACK = make_command('I', UBYTE)

COM_POKE_RECV = make_command('P')

COM_READ_TOF_RECV = make_command('T', UBYTE)
COM_READ_TOF_ACK = make_command('T', USHORT)

COM_SET_LED_RECV = make_command('L', UBYTE * 4)

COM_SET_GRIPPER_RECV = make_command('G', UBYTE)
COM_READ_GRIPPER_RECV = make_command('g')
COM_READ_GRIPPER_ACK = make_command('g', UBYTE)

board = Inventor2040W(init_encoders=False)
comms = USBSerialComms()

xshut_pins = [Pin(0, Pin.OUT), Pin(1, Pin.OUT), Pin(2, Pin.OUT)]

# Shut all the ToF sensors down
for xshut in xshut_pins:
    xshut.low()

tof_sensors = []

for i, xshut in enumerate(xshut_pins):
    xshut.high()
    try:
        vl53 = VL53L4CD(board.i2c)
        vl53.set_address(0x41 + 1 + i)

        #vl53.inter_measurement = 0
        vl53.timing_budget = 1000 // 10
    except OSError:
        vl53 = None
    tof_sensors.append(vl53)
    
if tof_sensors.count(None) > 0:
    for led in range(NUM_LEDS):
        board.leds.set_rgb(led, 64, 0, 64)
    while not board.switch_pressed():
        pass

for i in range(NUM_LEDS):
    board.leds.set_rgb(i, 64, 64, 64)

last_distance = [0] * len(tof_sensors)

gripper_servo_l = board.servos[0]
gripper_servo_r = board.servos[1]

def comms_connected():
    # Update all the LEDs
    for i in range(NUM_LEDS):
        board.leds.set_rgb(i, 0, 64, 0)
    for vl53 in tof_sensors:
        if vl53 is not None:
            vl53.start_ranging()

def comms_disconnected():
    # Update all the LEDs
    for i in range(NUM_LEDS):
        board.leds.set_rgb(i, 64, 0, 0)
    for vl53 in tof_sensors:
        if vl53 is not None:
            vl53.stop_ranging()
    gripper_servo_l.disable()
    gripper_servo_r.disable()

def identify_ack():
    comms.send(COM_IDENTIFY_ACK, "B", BOARD_ID)

def read_tof_ack(data):
    if data >= 0 and data < len(tof_sensors):
        comms.send(COM_READ_TOF_ACK, "H", int(last_distance[data] * 10))

def set_led(data):
    led, r, g, b = data
    if led < NUM_LEDS:
        board.leds.set_rgb(led, r, g, b)

GRIPPER_CLOSED = 0
GRIPPER_OPEN = 1
GRIPPER_UNKNOWN = 2
GRIPPER_CLOSING = 3
GRIPPER_OPENING = 4

SERVO_TIMESTEP = 20
SERVO_DURATION_MS = 1000

GRIPPER_OPEN_VALUE = 0
GRIPPER_CLOSED_VALUE = -50

gripper_state = GRIPPER_UNKNOWN

def set_gripper(state):
    if state == GRIPPER_OPEN:
        end_value = GRIPPER_OPEN_VALUE
        target_state = GRIPPER_OPEN
        gripper_state = GRIPPER_OPENING
    elif state == GRIPPER_CLOSED:
        end_value = GRIPPER_CLOSED_VALUE
        target_state = GRIPPER_CLOSED
        gripper_state = GRIPPER_CLOSING
    else:
        gripper_servo_l.disable()
        gripper_servo_r.disable()
        gripper_state = GRIPPER_UNKNOWN
        return

    gripper_servo_l.enable()
    gripper_servo_r.enable()

    time_elapsed = 0
    start_value_l = gripper_servo_l.value()
    start_value_r = gripper_servo_r.value()

    def update(timer):
        nonlocal time_elapsed
        global gripper_state
        time_elapsed += SERVO_TIMESTEP
        if time_elapsed >= SERVO_DURATION_MS:
            timer.deinit()  # Stop the timer when duration is reached
            gripper_servo_l.value(end_value)
            gripper_servo_r.value(-end_value)
            gripper_state = target_state
        else:
            gripper_servo_l.to_percent(time_elapsed, 0, SERVO_DURATION_MS, start_value_l, end_value)
            gripper_servo_r.to_percent(time_elapsed, 0, SERVO_DURATION_MS, start_value_r, -end_value)

    timer = Timer(-1)
    timer.init(period=SERVO_TIMESTEP, mode=Timer.PERIODIC, callback=update)

def read_gripper_ack():
    comms.send(COM_READ_GRIPPER_ACK, "B", gripper_state)

# Setup
comms.set_comms_established_callback(comms_connected)
comms.set_no_comms_timeout_callback(comms_disconnected)

comms.assign(COM_POKE_RECV)
comms.assign(COM_IDENTIFY_RECV, identify_ack)
comms.assign(COM_READ_TOF_RECV, read_tof_ack)
comms.assign(COM_SET_LED_RECV, set_led)
comms.assign(COM_SET_GRIPPER_RECV, set_gripper)
comms.assign(COM_READ_GRIPPER_RECV, read_gripper_ack)

# Credit to DrFootleg
time.sleep(5)  # Sleep to allow time to stop program on power up to get to repl
micropython.kbd_intr(-1)

comms_disconnected()

# TEMP
#for vl53 in tof_sensors:
#    vl53.start_ranging()
    
set_gripper(GRIPPER_CLOSED)
time.sleep(SERVO_DURATION_MS / 1000)
set_gripper(GRIPPER_OPEN)
time.sleep(SERVO_DURATION_MS / 1000)
set_gripper(GRIPPER_CLOSED)
time.sleep(SERVO_DURATION_MS / 1000)


tof_read_fails = [0] * len(tof_sensors)
tof_read_invalid = [0] * len(tof_sensors)
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


for i in range(NUM_LEDS):
    board.leds.set_rgb(i, 0, 0, 0)

gripper_servo_l.disable()
gripper_servo_r.disable()
