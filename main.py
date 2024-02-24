import time
import micropython
from inventor import Inventor2040W, NUM_LEDS
from comms.serial import make_command, USBSerialComms, UBYTE, USHORT
from sensors.vl53l4cd import VL53L4CD

BOARD_ID = 0x01

COM_IDENTIFY_RECV = make_command('I')
COM_IDENTIFY_ACK = make_command('I', UBYTE)

COM_POKE_RECV = make_command('P')

COM_READ_TOF_RECV = make_command('T')
COM_READ_TOF_ACK = make_command('T', USHORT)

COM_SET_LED_RECV = make_command('L', UBYTE * 4)

board = Inventor2040W(init_encoders=False)
comms = USBSerialComms()
vl53 = VL53L4CD(board.i2c)

#vl53.inter_measurement = 0
vl53.timing_budget = 1000 // 10

last_distance = 0

def comms_connected():
    # Update all the LEDs
    for i in range(NUM_LEDS):
        board.leds.set_rgb(i, 0, 64, 0)
    vl53.start_ranging()

def comms_disconnected():
    # Update all the LEDs
    for i in range(NUM_LEDS):
        board.leds.set_rgb(i, 64, 0, 0)
    vl53.stop_ranging()

def identify_ack():
    comms.send(COM_IDENTIFY_ACK, "B", BOARD_ID)

def read_tof_ack():
    comms.send(COM_READ_TOF_ACK, "H", int(last_distance * 10))

def set_led(data):
    led, r, g, b = data
    if led < NUM_LEDS:
        board.leds.set_rgb(led, r, g, b)


# Setup
comms.set_comms_established_callback(comms_connected)
comms.set_no_comms_timeout_callback(comms_disconnected)

comms.assign(COM_POKE_RECV)
comms.assign(COM_IDENTIFY_RECV, identify_ack)
comms.assign(COM_READ_TOF_RECV, read_tof_ack)
comms.assign(COM_SET_LED_RECV, set_led)

# Credit to DrFootleg
time.sleep(5)  # Sleep to allow time to stop program on power up to get to repl
micropython.kbd_intr(-1)

comms_disconnected()

while not board.switch_pressed():
    comms.check_receive()
    if vl53.data_ready:
        vl53.clear_interrupt()
        status = vl53.range_status()
        last_distance = vl53.distance
        #if status != 4:
            #print("Distance: {} cm, Status {}, Taken {} ms".format(vl53.distance, status, time.ticks_diff(end_time, start_time)))

for i in range(NUM_LEDS):
    board.leds.set_rgb(i, 0, 0, 0)
