
from enum import Enum
from .uart_log_packet import UARTLogPacket, OrientationDataLogPacket, CameraDataLogPacket
from .uart_handle_manager import STM32UARTFactory
import threading

class PacketID(Enum):
    ORIENTATION = 0xDEAD
    CAMERA = 0xBEEF

class UARTLogPacketFactory:

    @staticmethod
    def _create_packet_read_cb(e: Exception, data: bytes) -> UARTLogPacket:
        if e is not None:
            print(f"Exception occurred in read while creating packet: {e}")
            return None

        if len(data) < 4:
            raise ValueError("Failed to read complete metadata from UART")
        
        uart_mgr = STM32UARTFactory.get_manager()

        # Extract the packet ID
        packet_id = int.from_bytes(data[:2], byteorder='little')
        packet_size = int.from_bytes(data[2:], byteorder='little')

        # Create the appropriate packet instance based on the packet ID
        if packet_id == PacketID.ORIENTATION.value:
            return OrientationDataLogPacket(uart_mgr, packet_id, packet_size)
        elif packet_id == PacketID.CAMERA.value:
            return CameraDataLogPacket(uart_mgr, packet_id, packet_size)
        else:
            # If no specific subclass matches, return a generic UARTLogPacket
            return UARTLogPacket(uart_mgr, packet_id, packet_size)

    @staticmethod
    def create_packet() -> UARTLogPacket:
        """
        Factory method to create the appropriate UARTLogPacket subclass instance
        based on the packet ID read from the UART.
        :return: An instance of a subclass of UARTLogPacket.
        """
        # Read the first 4 bytes to determine the packet ID and size
        uart_mgr = STM32UARTFactory.get_manager()
        result = [None]  # Use a mutable container to capture the callback result

        event = threading.Event()

        def callback(e: Exception, data: bytes) -> None:
            result[0] = UARTLogPacketFactory._create_packet_read_cb(e, data)
            event.set()  # Signal that the callback has been invoked

        uart_mgr.request_read(4, callback)
        event.wait()  # Wait for the callback to complete

        if result[0] is None:
            raise RuntimeError("Failed to create packet")
        return result[0]
