import struct

from .uart_handle_manager import STM32UARTManager

class UARTLogPacket:
    def __init__(self, uart: STM32UARTManager, packet_id = 0xFFFF, packet_size = 0):
        """
        Initialize the UARTLogPacket with a UART handle.
        :param uart: A serial.Serial object representing the UART connection.
        """
        self.uart = uart
        self.packet_id = packet_id
        self.uart_packet_id = None
        self.packet_size = packet_size

        self.ready = False
    
    def __getitem__(self, key):
        # Allow accessing attributes of the object using dictionary-like syntax
        return getattr(self, key)

    def __setitem__(self, key, value):
        # Allow setting attributes of the object using dictionary-like syntax
        setattr(self, key, value)

    def _decode_data(self, e: Exception, raw_data: bytes):
        """
        Virtual function to decode the data in the packet.
        This function should be overridden by subclasses to implement specific decoding logic.
        :param raw_data: A bytes object containing the raw data.
        """
        raise NotImplementedError("Subclasses must implement this method to decode packet data")
    
    def is_ready(self):
        return self.ready

class OrientationDataLogPacket(UARTLogPacket):
    def __init__(self, uart: STM32UARTManager, packet_id, packet_size):
        super().__init__(uart, packet_id, packet_size)
        
        uart.request_read(self.packet_size, self._decode_data)

    def _decode_data(self, e: Exception, raw_data: bytes):
        """
        Decode the orientation data from the raw data.
        Assumes the data contains three 4-byte floats representing pitch, roll, and yaw.
        :param raw_data: A bytes object containing the raw data.
        """
        if(raw_data == None):
            return

        if len(raw_data) == 52:
            unpacked_data = struct.unpack(
                "<hhhhhhhhhhhhbbbbiiiiii",  # Little-endian format
                raw_data
            )

            self["out_x_g_raw"] = unpacked_data[0]
            self["out_y_g_raw"] = unpacked_data[1]
            self["out_z_g_raw"] = unpacked_data[2]
            self["out_x_a_raw"] = unpacked_data[3]
            self["out_y_a_raw"] = unpacked_data[4]
            self["out_z_a_raw"] = unpacked_data[5]
            self["out_x_raw_offboard"] = unpacked_data[6]
            self["out_y_raw_offboard"] = unpacked_data[7]
            self["out_z_raw_offboard"] = unpacked_data[8]
            self["out_x_raw_onboard"] = unpacked_data[9]
            self["out_y_raw_onboard"] = unpacked_data[10]
            self["out_z_raw_onboard"] = unpacked_data[11]
            self["microstep_mode"] = unpacked_data[12]
            self["_res1"] = unpacked_data[13] << 16 | unpacked_data[14] << 8 | unpacked_data[15]
            self["yaw_position"] = unpacked_data[16]
            self["roll_position"] = unpacked_data[17]
            self["pitch_position"] = unpacked_data[18]
            self["yaw_rate"] = unpacked_data[19]
            self["roll_rate"] = unpacked_data[20]
            self["pitch_rate"] = unpacked_data[21]
        else:
            raise ValueError("Invalid orientation data size")
        
        self.ready = True

class CameraDataLogPacket(UARTLogPacket):
    def __init__(self, uart: STM32UARTManager, packet_id, packet_size):
        super().__init__(uart, packet_id, packet_size)        
        uart.request_read(self.packet_size, self._decode_data)

    def _decode_data(self, e: Exception, raw_data: bytes):
        """
        Decode the camera data from the raw data.
        :param raw_data: A bytes object containing the raw data.
        """
        if(raw_data == None):
            raise ValueError("No RAW data to process!")

        # Extract the column information from the first 4 bytes of raw_data
        if len(raw_data) >= 4:
            self["last_packet"] = int.from_bytes(raw_data[:4], byteorder='little')
            self["camera_data"] = raw_data[4:]  # The rest of the data is image data
        else:
            raise ValueError("Insufficient data to extract column information")
        
        self.ready = True


