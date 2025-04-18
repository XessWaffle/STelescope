from queue import Queue, Empty

from .uart_handle_manager import STM32UARTFactory
from .uart_log_packet_factory import UARTLogPacketFactory
from .uart_log_packet import OrientationDataLogPacket, CameraDataLogPacket, UARTLogPacket
from threading import Lock

class UARTRxManager:
    def __init__(self):
        """
        Initializes the UARTRxManager with an STM32UARTManager instance.
        """
        self.uart_mgr = STM32UARTFactory.get_manager()
        self.packet_queue = Queue()
        self.invalid_packets = 0

    @staticmethod
    def _check_validity(packet):
        valid = isinstance(packet, OrientationDataLogPacket) or \
                isinstance(packet, CameraDataLogPacket)
        
        return valid

    def check_and_process_uart(self):
        """
        Checks if the STM32UARTManager has bytes to read, creates a packet, 
        and adds it to the internal queue with thread safety using a lock.
        """
        if self.invalid_packets > 10:
            self.invalid_packets = 0

            def callback(e: Exception):
                print(f"Flush Rx Callback: Exception {e}")

            self.uart_mgr.request_flush_rx(callback)
            return

        if self.uart_mgr.has_data_to_read():
            packet = UARTLogPacketFactory.create_packet()
            is_valid = UARTRxManager._check_validity(packet)
            #print(f"Packet ID: {hex(packet.packet_id)}, Packet Size: {packet.packet_size}, Is Valid: {is_valid}, Is Ready: {packet.is_ready()}")

            if not is_valid:
                self.invalid_packets += 1
                return
            
            self.packet_queue.put(packet)
            #print(f"Queue Length: {self.packet_queue.qsize()}")

    def poll_packet(self) -> tuple[type | None, UARTLogPacket | None]:
        """
        Retrieves and removes the next packet from the queue if available.
        Returns a tuple containing the type of the packet and the packet itself.
        Returns (None, None) if the queue is empty.
        """
        
        if(self.packet_queue.qsize() != 0):
            #print(f"Grabbed packet, Queue Length: {self.packet_queue.qsize()}")
            packet = self.packet_queue.get()
            return (type(packet), packet)
        else:
            return (None, None)

