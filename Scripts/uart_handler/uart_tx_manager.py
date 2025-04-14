from .uart_handle_manager import STM32UARTFactory
from .uart_command import UARTCommand

class UARTTxManager:
    def __init__(self):
        self.uart_mgr = STM32UARTFactory.get_manager()

    def _write_command_cb(self, e : Exception):
        if e is not None:
            print(f"Exception occurred during writing the command: {e}")

    def write_command(self, command : UARTCommand):
        # Ensure the command is not None and is an instance of UARTCommand
        if command is None:
            raise ValueError("Command cannot be None.")
        if not isinstance(command, UARTCommand):
            raise TypeError("Command must be an instance of UARTCommand.")
        
        self.uart_mgr.request_write(command.to_bytes(), self._write_command_cb)

