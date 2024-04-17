import serial
import struct
from collections import namedtuple
from time import monotonic_ns

START_BYTE = 13
FRAME_BYTES = 3

RECEIVE_TIMEOUT = 0.5

UBYTE = "B"
SBYTE = "b"
USHORT = "H"
SSHORT = "h"
FLOAT = "f"

def format_length(fmt):
    length = 0
    for char in fmt:
        if char in 'cbB?':
            length += 1  # char, signed/unsigned char, bool
        elif char in 'hH':
            length += 2  # short, unsigned short
        elif char in 'iI':
            length += 4  # int, unsigned int
        elif char in 'lL':
            length += 4 if struct.calcsize('l') == 4 else 8  # long, unsigned long
        elif char == 'f':
            length += 4  # float
        elif char == 'd':
            length += 8  # double
        else:
            raise ValueError(f"Unsupported format character: {char}")
    return length

Command = namedtuple("Command", ("value", "length", "format"))

def make_command(value, fmt=""):
    return Command(value, format_length(fmt), fmt)

COM_POKE_SEND = make_command('P')
COM_IDENTIFY_SEND = make_command('I')
COM_READ_TOF_SEND = make_command('T', UBYTE)
COM_READ_TOF_RECV = make_command('T', SSHORT)
COM_SET_LED_SEND = make_command('L', UBYTE * 4)

COM_SET_GRIPPER = make_command('G', UBYTE)
COM_READ_GRIPPER = make_command('g')


def checksum(buffer):
    checksum = 0
    for i in range(len(buffer) - 1):
        checksum += buffer[i]
    return checksum % 0x100

def send(ser, command: Command, *data):
    # Create a buffer of the correct length
    buffer = bytearray(command.length + FRAME_BYTES)

    # Populate the buffer with the required header values and command data
    struct.pack_into(">BB" + command.format + "B", buffer, 0,  # fmt, buffer, offset
                        START_BYTE,
                        ord(command.value),
                        *data,
                        0)  # where the checksum will go

    # Calculate the checksum and update the last byte of the buffer
    buffer[-1] = checksum(buffer)
    print("Sending: ", buffer, end="\t")
    # Send data to the serial port
    ser.write(buffer)


def receive(ser, command: Command, timeout=1):
    ms = int(1000.0 * timeout + 0.5)
    end_ms = (monotonic_ns() // 1000000) + ms

    # Wait for data of the expected size to be received
    receive_length = command.length + FRAME_BYTES
    while ser.in_waiting < receive_length:
        remaining_ms = end_ms - (monotonic_ns() // 1000000)

        # Has the timeout been reached?
        if remaining_ms <= 0:
            raise TimeoutError("Serial did not reply within the expected time")

    received = ser.read(receive_length)  # Read the expected number of bytes
    print(received)
    expected_checksum = checksum(received)
    received_checksum = received[-1]
    if received_checksum != expected_checksum:
        print("\n--------------------------------------------------")
        print("Recv Len:", end=" ")
        print(receive_length)
        print("Recv'd:", end=" ")
        print(received)
        print(received.hex())
        print("In Buffer:", end=" ")
        read_all = ser.read_all()
        print(read_all)
        print(read_all.hex())
        print("--------------------------------------------------")
        raise ValueError(f"Checksum mismatch! Expected {expected_checksum}, received {received_checksum}")

    else:
        print("")


if __name__ == "__main__":
    # Replace 'COMx' with your actual COM port, e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux
    com_port = 'COM7'

    ser = None
    try:
        # Open the serial port
        ser = serial.Serial(com_port, baudrate=9600, timeout=1)

        # Call the function to send and receive bytes
        while True:
            send(ser, COM_READ_TOF_SEND, 0)
            receive(ser, COM_READ_TOF_RECV, 1)           

    except serial.SerialException as e:
        print(f"Error: {e}")

    finally:
        # Close the serial port
        if ser is not None:
            ser.close()

