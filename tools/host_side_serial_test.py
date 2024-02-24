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
COM_READ_TOF_SEND = make_command('T')
COM_SET_LED_SEND = make_command('L', UBYTE * 4)


def checksum(buffer):
    checksum = 0
    for i in range(len(buffer) - 1):
        checksum += buffer[i]
    return checksum % 0x100

def create_data(command: Command, *data):
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
    return buffer


def send_and_receive_bytes(ser, data_to_send, timeout=1):
    print("Sending: ", data_to_send, end="\t")
    # Send data to the serial port
    ser.write(data_to_send)

    ms = int(1000.0 * timeout + 0.5)
    end_ms = (monotonic_ns() // 1000000) + ms

    # Wait for data of the expected size to be received
    while end_ms - (monotonic_ns() // 1000000) > 0:
        pass

    if ser.in_waiting > 0:
        # Read and print any bytes received
        received_data = ser.read_all()
        if len(received_data) > 0:
            print("Received: ", received_data)
    else:
        print("")


if __name__ == "__main__":
    # Replace 'COMx' with your actual COM port, e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux
    com_port = 'COM4'

    ser = None
    try:
        # Open the serial port
        ser = serial.Serial(com_port, baudrate=9600, timeout=1)

        # Call the function to send and receive bytes
        g = 0
        while True:
            #data_to_send = create_data(COM_READ_TOF_SEND)
            data_to_send = create_data(COM_SET_LED_SEND, 0, 255, g, 255)
            g = (g + 1) & 0xff
            send_and_receive_bytes(ser, bytes(data_to_send), RECEIVE_TIMEOUT)

    except serial.SerialException as e:
        print(f"Error: {e}")

    finally:
        # Close the serial port
        if ser is not None:
            ser.close()

