import serial
import threading
import queue
import serial.tools.list_ports

class STM32UARTHandle:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(STM32UARTHandle, cls).__new__(cls)
        return cls._instance

    def __init__(self, baudrate=115200, timeout=1):
        if not hasattr(self, '_initialized'):
            self.baudrate = baudrate
            self.timeout = timeout
            self.uart_handle = None
            self.port = self._find_stm32_port()
            if self.port:
                self.open_connection()
            self._initialized = True

    def __call__(self) -> serial.Serial:
        """Return the UART handle when the instance is called."""
        return self._get_uart_handle()

    def __del__(self) -> None:
        """Ensure the connection is closed when the object is deleted."""
        self.close_connection()

    def _find_stm32_port(self) -> str:
        """Find the COM port associated with the STM32 device."""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "STM32" in port.description or "USB Serial Device" in port.description:  # Adjust this string if needed
                return port.device
        raise Exception("STM32 UART COM port not found.")
    
    def _get_uart_handle(self) -> serial.Serial:
        """Get the UART handle."""
        if self.uart_handle and self.uart_handle.is_open:
            return self.uart_handle
        raise Exception("UART connection is not open.")

    def open_connection(self) -> None:
        """Open the UART connection."""
        if self.uart_handle is None:
            self.uart_handle = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
        elif not self.uart_handle.is_open:
            self.uart_handle.open()

    def close_connection(self) -> None:
        """Close the UART connection."""
        if self.uart_handle and self.uart_handle.is_open:
            self.uart_handle.close()
    
    def available(self) -> int:
        """Return the number of bytes available to read."""
        if self.uart_handle and self.uart_handle.is_open:
            return self.uart_handle.in_waiting
        raise Exception("UART connection is not open.")
    
    def read(self, num: int) -> bytes:
        """Read the specified number of bytes from the UART connection."""
        if self.uart_handle and self.uart_handle.is_open:
            return self.uart_handle.read(num)
        raise Exception("UART connection is not open.")

    def write(self, data: bytes) -> None:
        """Write the specified bytes to the UART connection."""
        if self.uart_handle and self.uart_handle.is_open:
            self.uart_handle.write(data)
        else:
            raise Exception("UART connection is not open.")
        
    def flush_rx(self) -> None:
        """Flush the RX buffer of the UART connection."""
        if self.uart_handle and self.uart_handle.is_open:
            self.uart_handle.reset_input_buffer()
        else:
            raise Exception("UART connection is not open.")
        
class STM32UARTManager:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(STM32UARTManager, cls).__new__(cls)
        return cls._instance

    def __init__(self, uart_handle: STM32UARTHandle):
        if not hasattr(self, '_initialized'):
            self.uart_handle = uart_handle
            self.read_queue = queue.Queue()
            self.write_queue = queue.Queue()
            self.read_thread = threading.Thread(target=self._process_reads, daemon=True)
            self.write_thread = threading.Thread(target=self._process_writes, daemon=True)
            self.read_thread.start()
            self.write_thread.start()
            self._initialized = True
    
    def has_data_to_read(self) -> bool:
        """Check if there are bytes available to read."""
        try:
            return self.uart_handle.available() > 0
        except Exception:
            return False
        
    def request_read(self, num: int, callback) -> None:
        """Request a read operation with a callback."""
        self.read_queue.put((num, callback))

    def request_write(self, data: bytes, callback) -> None:
        """Request a write operation with a callback."""
        self.write_queue.put((data, callback))
    
    def request_flush_rx(self, callback) -> None:
        """Request a flush of the RX buffer with a callback."""
        self.read_queue.put((0, lambda e, _: callback(e)))

    def _process_reads(self) -> None:
        """Process read requests from the queue."""
        while True:
            num, callback = self.read_queue.get()
            try:
                data = None
                if num == 0: # Indicates flush request
                    self.uart_handle.flush_rx()
                else:
                    data = self.uart_handle.read(num)

                callback(None, data)  # Pass data to the callback
            except Exception as e:
                callback(e, None)  # Pass the exception to the callback
            finally:
                self.read_queue.task_done()

    def _process_writes(self) -> None:
        """Process write requests from the queue."""
        while True:
            data, callback = self.write_queue.get()
            try:
                self.uart_handle.write(data)
                callback(None)  # Indicate success to the callback
            except Exception as e:
                callback(e)  # Pass the exception to the callback
            finally:
                self.write_queue.task_done()


class STM32UARTFactory:
    @staticmethod
    def get_manager() -> STM32UARTManager:
        """Create and return an instance of STM32UARTManager."""
        uart_handle = STM32UARTHandle()
        return STM32UARTManager(uart_handle)