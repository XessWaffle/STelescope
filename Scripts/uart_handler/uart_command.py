from enum import Enum

class UARTCommands(Enum):
    CMD_R_PLUS = 0x01  # Move the rotation axis with a positive rate
    CMD_R_MINUS = 0x02  # Move the rotation axis with a negative rate
    CMD_R_STOP = 0x03  # Stop the rotation axis
    CMD_Y_PLUS = 0x04  # Move the yaw axis with a positive rate
    CMD_Y_MINUS = 0x05  # Move the yaw axis with a negative rate
    CMD_Y_STOP = 0x06  # Stop the yaw axis
    CMD_P_PLUS = 0x07  # Move the pitch axis with a positive rate
    CMD_P_MINUS = 0x08  # Move the pitch axis with a negative rate
    CMD_P_STOP = 0x09  # Stop the pitch axis
    CMD_SET_STATE = 0x0A  # Set the state
    CMD_SET_RATE = 0x0B  # Set the rate
    CMD_RESET_POS = 0x0C  # Reset the position
    CMD_HOME = 0x0D  # Deprecated
    CMD_MICST_MODE = 0x0E  # Change the microstep mode (mode given in info)
    CMD_CAPTURE = 0x0F  # Capture
    CMD_EM_STOP = 0x10  # Emergency stop
    CMD_STOP = 0x11     # Regular stop
    CMD_ACK = 0x12      # Acknowledge
    CMD_INVALID = 0x13  # Invalid command


class UARTCommand:

    def __init__(self, command_byte=UARTCommands.CMD_INVALID.value, info_bytes=None):
        """
        Initializes the UARTCommand object.

        :param command_byte: The first byte representing the command, either as an int or from the UARTCommands enum (default is CMD_INVALID).
        :param info_bytes: A list or tuple of three bytes representing additional info (default is [0x00, 0x00, 0x00]).
        """
        if isinstance(command_byte, UARTCommands):
            self.command_byte = command_byte.value
        elif isinstance(command_byte, int):
            self.command_byte = command_byte
        else:
            raise TypeError("command_byte must be an int or a UARTCommands enum value.")
        
        if isinstance(info_bytes, int):
            if not (0 <= info_bytes <= 0xFFFFFF):
                raise ValueError("Info bytes as an integer must be between 0x000000 and 0xFFFFFF.")
            self.info_bytes = [(info_bytes >> 16) & 0xFF, (info_bytes >> 8) & 0xFF, info_bytes & 0xFF]
        else:
            self.info_bytes = info_bytes if info_bytes is not None else [0x00, 0x00, 0x00]
        self._validate()

    def _validate(self) -> None:
        """Validates the command byte and info bytes."""
        if not (0x00 <= self.command_byte <= UARTCommands.CMD_INVALID.value):
            raise ValueError("Command byte must be between 0x00 and 0xFF.")
        if len(self.info_bytes) != 3 or not all(0x00 <= b <= 0xFF for b in self.info_bytes):
            raise ValueError("Info bytes must be a list or tuple of three bytes between 0x00 and 0xFF.")

    def set_command_byte(self, command_byte: int | UARTCommands) -> None:
        """Sets the command byte."""
        if isinstance(command_byte, UARTCommands):
            self.command_byte = command_byte.value
        elif isinstance(command_byte, int) and (0x00 <= command_byte <= 0xFF):
            self.command_byte = command_byte
        else:
            raise ValueError("Command byte must be an int between 0x00 and 0xFF or a UARTCommands enum value.")

    def set_info_bytes(self, info_bytes: list[int] | tuple[int, int, int]) -> None:
        """Sets the info bytes."""
        if len(info_bytes) != 3 or not all(0x00 <= b <= 0xFF for b in info_bytes):
            raise ValueError("Info bytes must be a list or tuple of three bytes between 0x00 and 0xFF.")
        self.info_bytes = info_bytes

    def get_command_byte(self) -> UARTCommands:
        """Returns the command byte as a UARTCommands enum if possible, otherwise returns CMD_INVALID."""
        try:
            return UARTCommands(self.command_byte)
        except ValueError:
            return UARTCommands.CMD_INVALID

    def get_info_bytes(self) -> list[int]:
        """Returns the info bytes."""
        return self.info_bytes

    def to_bytes(self, endian: str = "big") -> bytes:
        """
        Returns the full 4-byte command as a bytes object.

        :param endian: The byte order, either "big" or "little". Default is "big".
        :return: A bytes object representing the command.
        """
        if endian not in ("big", "little"):
            raise ValueError('Endian must be either "big" or "little".')
        
        if endian == "big":
            return bytes([self.command_byte] + self.info_bytes)
        else:  # little endian
            return bytes(self.info_bytes[::-1] + [self.command_byte])

    @classmethod
    def from_bytes(cls, byte_data: bytes) -> "UARTCommand":
        """
        Creates a UARTCommand object from a 4-byte sequence.

        :param byte_data: A bytes object of length 4.
        :return: A UARTCommand instance.
        """
        if len(byte_data) != 4:
            raise ValueError("Byte data must be exactly 4 bytes long.")
        return cls(command_byte=byte_data[0], info_bytes=list(byte_data[1:]))