from inventor import Inventor2040W, NUM_LEDS
from comms.serial import Command, USBSerialComms

BOARD_ID = 0x01

COM_IDENTIFY_RECV = Command('I', 0)
COM_IDENTIFY_ACK = Command('I', 1)

COM_POKE_RECV = Command('P', 0)

board = Inventor2040W()
comms = USBSerialComms()

def comms_connected():
    # Update all the LEDs
    for i in range(NUM_LEDS):
        board.leds.set_rgb(i, 0, 64, 0)

def comms_disconnected():
    # Update all the LEDs
    for i in range(NUM_LEDS):
        board.leds.set_rgb(i, 64, 0, 0)

def identify_ack():
    comms.send(COM_IDENTIFY_ACK, "B", BOARD_ID)


# Setup
comms.set_comms_established_callback(comms_connected)
comms.set_no_comms_timeout_callback(comms_disconnected)
comms.assign(COM_IDENTIFY_RECV, identify_ack)
comms.assign(COM_POKE_RECV)

comms_disconnected()
while True:
    comms.check_receive()
