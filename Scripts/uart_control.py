import serial
from pynput import keyboard

import serial.tools.list_ports

from queue import Queue
import pygame

active_keys = []
cmds = Queue()

CMD_R_PLUS = 0
CMD_R_MINUS = 1
CMD_R_STOP = 2
CMD_Y_PLUS = 3
CMD_Y_MINUS = 4
CMD_Y_STOP = 5
CMD_P_PLUS = 6
CMD_P_MINUS = 7
CMD_P_STOP = 8
CMD_HOME = 9
CMD_MICST_MODE = 10
CMD_INVALID = 11

microstep_mode = 0

def find_stm32_port():
    """Find the serial port associated with an STM32 device."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "USB Serial Device" in port.description:
            return port.device
    return None

def open_uart_connection(port, baudrate, bytesize, parity, stopbits, timeout):
    """Open a UART connection with the specified parameters."""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=timeout
        )
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None

def handle_key_event(event, is_pressed):
    """Handle key press or release events."""
    key = event.key
    cmd = None
    global microstep_mode

    if is_pressed:
        if key in active_keys:
            return
        print(f"Key pressed: {pygame.key.name(key)}")
        active_keys.append(key)

        match key:
            case pygame.K_q:
                cmd = CMD_R_PLUS << 24 | 0 << 16 | 0 << 8 | 0
            case pygame.K_w:
                cmd = CMD_P_PLUS << 24 | 0 << 16 | 0 << 8 | 0
            case pygame.K_e:
                cmd = CMD_R_MINUS << 24 | 0 << 16 | 0 << 8 | 0
            case pygame.K_a:
                cmd = CMD_Y_PLUS << 24 | 0 << 16 | 0 << 8 | 0
            case pygame.K_s:
                cmd = CMD_P_MINUS << 24 | 0 << 16 | 0 << 8 | 0
            case pygame.K_d:
                cmd = CMD_Y_MINUS << 24 | 0 << 16 | 0 << 8 | 0
            case pygame.K_h:
                cmd = CMD_HOME << 24 | 0 << 16 | 0 << 8 | 0
            case pygame.K_m:
                cmd = CMD_MICST_MODE << 24 | 0 << 16 | 0 << 8 | microstep_mode
                microstep_mode = microstep_mode + 1
                microstep_mode = microstep_mode % 5
            case _:
                cmd = None
    else:
        print(f"Key released: {pygame.key.name(key)}")
        if key in active_keys:
            active_keys.remove(key)

        match key:
            case pygame.K_q:
                cmd = CMD_R_STOP << 24 | 0 << 16 | 0 << 8 | 0
            case pygame.K_w:
                cmd = CMD_P_STOP << 24 | 0 << 16 | 0 << 8 | 0
            case pygame.K_e:
                cmd = CMD_R_STOP << 24 | 0 << 16 | 0 << 8 | 0
            case pygame.K_a:
                cmd = CMD_Y_STOP << 24 | 0 << 16 | 0 << 8 | 0
            case pygame.K_s:
                cmd = CMD_P_STOP << 24 | 0 << 16 | 0 << 8 | 0
            case pygame.K_d:
                cmd = CMD_Y_STOP << 24 | 0 << 16 | 0 << 8 | 0
            case _:
                cmd = None

    if cmd is not None:
        cmds.put(cmd)

def draw_ui(screen, font, active_keys, last_cmd):
    """Draw the UI to visualize active keys and last command."""
    screen.fill((0, 0, 0))  # Clear screen with black background

    # Display active keys
    active_keys_text = f"Active Keys: {', '.join([pygame.key.name(k) for k in active_keys])}"
    active_keys_surface = font.render(active_keys_text, True, (255, 255, 255))
    screen.blit(active_keys_surface, (10, 10))

    # Display last command
    last_cmd_text = f"Last Command: {last_cmd}"
    last_cmd_surface = font.render(last_cmd_text, True, (255, 255, 255))
    screen.blit(last_cmd_surface, (10, 50))

    pygame.display.flip()  # Update the display

def main():

    # UART parameters
    baudrate = 115200
    bytesize = serial.EIGHTBITS
    parity = serial.PARITY_NONE
    stopbits = serial.STOPBITS_ONE
    timeout = 5  # seconds

    # Find STM32 port
    stm32_port = find_stm32_port()
    if not stm32_port:
        print("STM32 device not found.")
        return

    print(f"STM32 device found on port: {stm32_port}")

    # Open UART connection
    uart = open_uart_connection(stm32_port, baudrate, bytesize, parity, stopbits, timeout)
    if not uart:
        return

    print("UART connection opened successfully.")

    # Start listening to keyboard events
    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("UART Control")
    font = pygame.font.Font(None, 36)

    running = True
    last_cmd = "None"
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                handle_key_event(event, is_pressed=True)
            elif event.type == pygame.KEYUP:
                handle_key_event(event, is_pressed=False)

        if not cmds.empty():
            cmd = cmds.get()
            last_cmd = f"0x{cmd:08X}"  # Format command as hex
            try:
                # Example: Write and read data
                byte_data = cmd.to_bytes(4, byteorder='big')     
                uart.write(byte_data)
            except Exception as e:
                print(f"Error during UART communication: {e}")
                running = False

        draw_ui(screen, font, active_keys, last_cmd)

    pygame.quit()

if __name__ == "__main__":
    main()