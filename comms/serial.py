import sys
import select
import struct
from collections import namedtuple
from time import ticks_ms, ticks_diff

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


class USBSerialComms:
    START_BYTE = 13
    FRAME_BYTES = 3

    WAITING = 0
    COMMAND_NEXT = 1
    DATA_NEXT = 2
    CHECKSUM = 3

    DEFAULT_TIMEOUT = 1

    def __init__(self, no_comms_timeout=DEFAULT_TIMEOUT):
        self.__stdin_poll = select.poll()        # Set up an input polling object.
        self.__stdin_poll.register(sys.stdin, select.POLLIN)    # Register polling object.

        # Receivable commands, the amount of data associated and what to do with it
        self.__commands = {}

        self.__no_comms_timeout_ms = int(1000.0 * no_comms_timeout + 0.5)
        self.__last_received_ms = 0
        self.__comms_established_callback = None
        self.__no_comms_timeout_callback = None
        self.__timeout_reached = True  # Set as true initially so timeout callback does not get called

        self.__rx_state = self.WAITING
        self.__rx_command = None
        self.__rx_callback = None
        self.__rx_data = []

    def assign(self, command, callback=None):
        self.__commands[ord(command.value)] = [command, callback]

    def __del__(self):
        self.__stdin_poll.unregister(sys.stdin)

    def is_connected(self):
        return self.__timeout_reached

    def check_receive(self):
        while self.__stdin_poll.poll(0):
            received = ord(sys.stdin.buffer.read(1))  # Read one byte, although it comes in as a list of one byte
            #print(received)

            if self.__rx_state == self.WAITING:  # Finished processing last command
                if received == self.START_BYTE:  # Start byte received
                    #print("Frame Started")
                    self.__rx_state = self.COMMAND_NEXT

            elif self.__rx_state == self.COMMAND_NEXT:
                self.__rx_command = None
                if received in self.__commands.keys():  # If command matters
                    self.__rx_command = self.__commands[received][0]
                    self.__rx_callback = self.__commands[received][1]
                    #print(f"{self.__rx_command}")
                    if self.__rx_command.length > 0:  # If there's data to look at
                        self.__rx_state = self.DATA_NEXT
                    else:
                        self.__rx_state = self.CHECKSUM

                if self.__rx_command is None:
                    self.__return_to_waiting()

            elif self.__rx_state == self.DATA_NEXT:
                self.__rx_data.append(received)  # Shove that data in a list
                #print(f"Adding Byte {received}")

                if len(self.__rx_data) == self.__rx_command.length:  # Finished receiving data
                    self.__rx_state = self.CHECKSUM

                elif len(self.__rx_data) > self.__rx_command.length:
                    self.__return_to_waiting()

            elif self.__rx_state == self.CHECKSUM:
                checksum_expected = USBSerialComms.checksum(self.__rx_command, self.__rx_data)
                #print("Checksum =", checksum_expected)
                if received == checksum_expected:  # Check that checksum
                    #print("Checksum passed!")
                    if self.__timeout_reached:
                        if self.__comms_established_callback is not None:
                            self.__comms_established_callback()
                        self.__timeout_reached = False

                    if self.__rx_callback is not None:
                        buffer = struct.unpack(">" + self.__rx_command.format, bytearray(self.__rx_data))
                        fmt_len = len(self.__rx_command.format)
                        if fmt_len > 0:
                            self.__rx_callback(buffer[0] if fmt_len == 1 else buffer)  # Call the referenced command
                        else:
                            self.__rx_callback()
                    self.__last_received_ms = ticks_ms()
                #else:
                    #print("Checksum failed!")

                self.__return_to_waiting()

        if (ticks_diff(ticks_ms(), self.__last_received_ms) > self.__no_comms_timeout_ms) and not self.__timeout_reached:
            if self.__no_comms_timeout_callback is not None:
                self.__no_comms_timeout_callback()
            self.__timeout_reached = True
            #print("Timeout Reached")

    def __return_to_waiting(self):
        self.__rx_command = None
        self.__rx_callback = None
        self.__rx_data = []
        self.__rx_state = self.WAITING
        #print("Return To Waiting")

    @staticmethod
    def checksum(command, buffer):
        checksum = USBSerialComms.START_BYTE + ord(command.value)
        for i in buffer:
            checksum += i
        return checksum % 0x100

    @staticmethod
    def checksum_from_full(buffer):
        checksum = 0
        for i in range(len(buffer) - 1):
            checksum += buffer[i]
        return checksum % 0x100

    def set_no_comms_timeout_callback(self, callback):
        self.__no_comms_timeout_callback = callback

    def set_comms_established_callback(self, callback):
        self.__comms_established_callback = callback

    def send(self, command: Command, fmt="", *data):
        # Create a buffer of the correct length
        buffer = bytearray(command.length + self.FRAME_BYTES)

        # Populate the buffer with the required header values and command data
        struct.pack_into(">BB" + fmt + "B", buffer, 0,  # fmt, buffer, offset
                         self.START_BYTE,
                         ord(command.value),
                         *data,
                         0)  # where the checksum will go

        # Calculate the checksum and update the last byte of the buffer
        buffer[-1] = self.checksum_from_full(buffer)

        # Write out the buffer, using method that doesn't touch carrage returns
        sys.stdout.buffer.write(buffer)
