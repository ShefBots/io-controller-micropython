import serial
import struct
from collections import namedtuple
from time import monotonic_ns

START_BYTE = 13
FRAME_BYTES = 3

RECEIVE_TIMEOUT = 0.5

Command = namedtuple("Command", ("value", "length"))
COM_POKE_SEND = Command('P', 0)
COM_IDENTIFY_SEND = Command('I', 0)
COM_READ_TOF_SEND = Command('T', 0)


def checksum(buffer):
    checksum = 0
    for i in range(len(buffer) - 1):
        checksum += buffer[i]
    return checksum % 0x100

def create_data(command: Command, fmt="", *data):
    # Create a buffer of the correct length
    buffer = bytearray(command.length + FRAME_BYTES)

    # Populate the buffer with the required header values and command data
    struct.pack_into(">BB" + fmt + "B", buffer, 0,  # fmt, buffer, offset
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
        # Read and print any bytes received
        received_data = ser.read_all()
        if len(received_data) > 0:
            print("Received: ", received_data)


if __name__ == "__main__":
    # Replace 'COMx' with your actual COM port, e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux
    com_port = 'COM4'

    ser = None
    try:
        # Open the serial port
        ser = serial.Serial(com_port, baudrate=9600, timeout=1)

        # Call the function to send and receive bytes
        while True:
            data_to_send = create_data(COM_READ_TOF_SEND)
            send_and_receive_bytes(ser, bytes(data_to_send), RECEIVE_TIMEOUT)

    except serial.SerialException as e:
        print(f"Error: {e}")

    finally:
        # Close the serial port
        if ser is not None:
            ser.close()

