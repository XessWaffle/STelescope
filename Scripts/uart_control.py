import serial

import serial.tools.list_ports

from queue import Queue
import pygame
import struct
import threading

class CameraDataPacket:
    def __init__(self, last_packet, size, bytes):
        self.last_packet = last_packet
        self.size = size
        self.bytes = bytes

class CameraData:
    def __init__(self):
        self.image_raw = [0] * (320 * 240 * 3)  # Initialize with zeros for a 320x240 RGB image
        self.image_captured = False

    def add_bytes(self, cdp : CameraDataPacket):
        
        idx = int((cdp.size * cdp.last_packet / 2) * 3)

        if(idx > len(self.image_raw) - 3):
            return

        # Convert each pair of bytes (RGB565) into three bytes (RGB888)
        for i in range(0, len(cdp.bytes), 2):
            if i + 1 < len(cdp.bytes) and idx < 320 * 240 * 3:
                rgb565 = (cdp.bytes[i] << 8) | cdp.bytes[i + 1]
                self.image_raw[idx + 0] = ((rgb565 >> 11) & 0x1F) * 255 // 31
                self.image_raw[idx + 1] = ((rgb565 >> 5) & 0x3F) * 255 // 63
                self.image_raw[idx + 2] = (rgb565 & 0x1F) * 255 // 31

                idx = idx + 3
    
    def __call__(self, *args, **kwds):

        # Ensure the image buffer has exactly enough elements for a 320x240 image (RGB format)
        return bytes(self.image_raw)

class OrientationPacket:
    def __init__(self, _res1, out_x_g_raw, out_y_g_raw, out_z_g_raw, out_x_a_raw, out_y_a_raw, out_z_a_raw,
                 out_x_raw_offboard, out_y_raw_offboard, out_z_raw_offboard, out_x_raw_onboard, out_y_raw_onboard,
                 out_z_raw_onboard, _res2, yaw_position, roll_position, pitch_position, yaw_rate, roll_rate, pitch_rate):
        self._res1 = _res1
        self.out_x_g_raw = out_x_g_raw
        self.out_y_g_raw = out_y_g_raw
        self.out_z_g_raw = out_z_g_raw
        self.out_x_a_raw = out_x_a_raw
        self.out_y_a_raw = out_y_a_raw
        self.out_z_a_raw = out_z_a_raw
        self.out_x_raw_offboard = out_x_raw_offboard
        self.out_y_raw_offboard = out_y_raw_offboard
        self.out_z_raw_offboard = out_z_raw_offboard
        self.out_x_raw_onboard = out_x_raw_onboard
        self.out_y_raw_onboard = out_y_raw_onboard
        self.out_z_raw_onboard = out_z_raw_onboard
        self._res2 = _res2
        self.yaw_position = yaw_position
        self.roll_position = roll_position
        self.pitch_position = pitch_position
        self.yaw_rate = yaw_rate
        self.roll_rate = roll_rate
        self.pitch_rate = pitch_rate

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
orientation_packet = None
camera_data = CameraData()

def find_stm32_port():
    """Find the serial port associated with an STM32 device."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(port.description)
        if "USB Serial Device" in port.description or "STM32" in port.description:
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

def handle_camera_metadata_packet(uart : serial.Serial):
    global camera_data

    print("Received packet: CAMERA_METADATA")
    # Read the next 54 bytes to flush the orientation packet
    camera_data_packet_bytes = uart.read(6)
    if len(camera_data_packet_bytes) == 6:
        print("Camera metadata packet flushed.")
    else:
        print(f"Failed to flush camera metadata packet. Received {len(camera_data_packet_bytes)} bytes.")

    # Unpack the 54 bytes into the specified struct

    try:
        unpacked_data = struct.unpack(
            "<hI",  # Little-endian format
            camera_data_packet_bytes
        )

        image_bytes = uart.read(unpacked_data[1])

        def process_camera_data():
            # Assign unpacked values to variables
            camera_data_packet = CameraDataPacket(
            last_packet=unpacked_data[0],
            size=unpacked_data[1],
            bytes=image_bytes
            )

            print(f"Camera Metadata Packet - Last Packet: {camera_data_packet.last_packet}, Size: {camera_data_packet.size}, Validate: {len(camera_data_packet.bytes)}")

            camera_data.add_bytes(camera_data_packet)

        # Start a new thread to process the camera data
        camera_thread = threading.Thread(target=process_camera_data)
        camera_thread.start()


    except struct.error as e:
        print(f"Error unpacking camera_metadata packet: {e}")


def handle_orientation_packet(uart):

    global orientation_packet

    print("Received packet: ORIENTATION")
    # Read the next 54 bytes to flush the orientation packet
    orientation_packet_bytes = uart.read(54)
    if len(orientation_packet_bytes) == 54:
        print("Orientation packet flushed.")
    else:
        print(f"Failed to flush orientation packet. Received {len(orientation_packet_bytes)} bytes.")

    # Unpack the 54 bytes into the specified struct

    try:
        unpacked_data = struct.unpack(
            "<HhhhhhhhhhhhhIiiiiii",  # Little-endian format
            orientation_packet_bytes
        )

        # Assign unpacked values to variables
        orientation_packet = OrientationPacket(
            _res1=unpacked_data[0],
            out_x_g_raw=unpacked_data[1],
            out_y_g_raw=unpacked_data[2],
            out_z_g_raw=unpacked_data[3],
            out_x_a_raw=unpacked_data[4],
            out_y_a_raw=unpacked_data[5],
            out_z_a_raw=unpacked_data[6],
            out_x_raw_offboard=unpacked_data[7],
            out_y_raw_offboard=unpacked_data[8],
            out_z_raw_offboard=unpacked_data[9],
            out_x_raw_onboard=unpacked_data[10],
            out_y_raw_onboard=unpacked_data[11],
            out_z_raw_onboard=unpacked_data[12],
            _res2=unpacked_data[13],
            yaw_position=unpacked_data[14],
            roll_position=unpacked_data[15],
            pitch_position=unpacked_data[16],
            yaw_rate=unpacked_data[17],
            roll_rate=unpacked_data[18],
            pitch_rate=unpacked_data[19]
        )

    except struct.error as e:
        print(f"Error unpacking orientation packet: {e}")

def handle_uart_rx(uart : serial.Serial):
    # Check if there are bytes available to read from UART
    if uart.in_waiting > 0:
        try:
            # Read the first two bytes from UART
            data = uart.read(2)
            if len(data) == 2:
                # Combine the two bytes into a single 16-bit value
                value = int.from_bytes(data, byteorder='little')
                
                # Handle the value with a switch-case-like structure
                match value:
                    case 0xDEAD:
                        handle_orientation_packet(uart)
                    case 0xBEEF:
                        handle_camera_metadata_packet(uart)
                    case _:
                        print(f"Unknown packet received: 0x{value:04X}")
        except Exception as e:
            print(f"Error reading from UART: {e}")
            return False
        
    return True

def handle_uart_tx(uart : serial.Serial):

    last_cmd = "None"

    if not cmds.empty():
        cmd = cmds.get()
        last_cmd = f"0x{cmd:08X}"  # Format command as hex
        try:
            # Example: Write and read data
            byte_data = cmd.to_bytes(4, byteorder='big')     
            uart.write(byte_data)
        except Exception as e:
            print(f"Error during UART communication: {e}")
            return (last_cmd, False)

    return (last_cmd, True)
    

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

    # Display orientation packet information if available
    if orientation_packet:
        # Draw a labeled bar graph for orientation angles
        bar_width = 200
        bar_height = 20
        base_x = 10
        base_y = 90
        max_steps = (360 / 1.8) * 1620 * (1 << microstep_mode)  # Assuming angles are in degrees
        spacing = 40  # Spacing between elements

        # Yaw
        yaw_width = int((orientation_packet.yaw_position / max_steps) * bar_width)
        pygame.draw.rect(screen, (255, 0, 0), (base_x, base_y, yaw_width, bar_height))
        yaw_text = font.render(f"Yaw: {orientation_packet.yaw_position}", True, (255, 255, 255))
        screen.blit(yaw_text, (base_x + bar_width + 10, base_y))

        # Roll
        roll_width = int((orientation_packet.roll_position / max_steps) * bar_width)
        pygame.draw.rect(screen, (0, 255, 0), (base_x, base_y + spacing, roll_width, bar_height))
        roll_text = font.render(f"Roll: {orientation_packet.roll_position}", True, (255, 255, 255))
        screen.blit(roll_text, (base_x + bar_width + 10, base_y + spacing))

        # Pitch
        pitch_width = int((orientation_packet.pitch_position / max_steps) * bar_width)
        pygame.draw.rect(screen, (0, 0, 255), (base_x, base_y + 2 * spacing, pitch_width, bar_height))
        pitch_text = font.render(f"Pitch: {orientation_packet.pitch_position}", True, (255, 255, 255))
        screen.blit(pitch_text, (base_x + bar_width + 10, base_y + 2 * spacing))

        # Display gyroscope data as a radar-like visualization
        center_x, center_y = 150, 400
        radius = 100
        pygame.draw.circle(screen, (255, 255, 255), (center_x, center_y), radius, 1)
        pygame.draw.line(screen, (255, 0, 0), (center_x, center_y),
                         (center_x + int(orientation_packet.out_x_g_raw / 32768 * radius),
                          center_y - int(orientation_packet.out_y_g_raw / 32768 * radius)), 2)
        gyro_text = font.render("Gyro", True, (255, 255, 255))
        screen.blit(gyro_text, (center_x - 20, center_y + radius + 10))

        # Display accelerometer data as a radar-like visualization
        accel_center_y = center_y + 2 * radius + 50  # Adjusted to avoid overlap
        pygame.draw.circle(screen, (255, 255, 255), (center_x, accel_center_y), radius, 1)
        pygame.draw.line(screen, (0, 255, 0), (center_x, accel_center_y),
                         (center_x + int(orientation_packet.out_x_a_raw / 32768 * radius),
                          accel_center_y - int(orientation_packet.out_y_a_raw / 32768 * radius)), 2)
        accel_text = font.render("Accel", True, (255, 255, 255))
        screen.blit(accel_text, (center_x - 20, accel_center_y + radius + 10))

        # Display onboard magnetometer data as a radar-like visualization
        onboard_center_x = center_x + 300
        pygame.draw.circle(screen, (255, 255, 255), (onboard_center_x, center_y), radius, 1)
        pygame.draw.line(screen, (0, 0, 255), (onboard_center_x, center_y),
                         (onboard_center_x + int(orientation_packet.out_x_raw_onboard / 5000 * radius),
                          center_y - int(orientation_packet.out_y_raw_onboard / 5000 * radius)), 2)
        onboard_text = font.render("Onboard Mag", True, (255, 255, 255))
        screen.blit(onboard_text, (onboard_center_x - 20, center_y + radius + 10))

        # Display offboard magnetometer data as a radar-like visualization
        offboard_center_y = accel_center_y  # Align with accelerometer visualization
        pygame.draw.circle(screen, (255, 255, 255), (onboard_center_x, offboard_center_y), radius, 1)
        pygame.draw.line(screen, (255, 255, 0), (onboard_center_x, offboard_center_y),
                         (onboard_center_x + int(orientation_packet.out_x_raw_offboard / 5000 * radius),
                          offboard_center_y - int(orientation_packet.out_y_raw_offboard / 5000 * radius)), 2)
        offboard_text = font.render("Offboard Mag", True, (255, 255, 255))
        screen.blit(offboard_text, (onboard_center_x - 20, offboard_center_y + radius + 10))

    # Convert the image data to a pygame surface and draw it
    image_data = camera_data()
    if image_data:
        try:
            # Convert the byte array to a pygame surface
            image_surface = pygame.image.frombuffer(
                image_data, (320, 240), "RGB"
            )
            # Scale the image to fit the screen
            scaled_image = pygame.transform.scale(image_surface, (640, 480))
            # Rotate the image 90 degrees
            rotated_image = pygame.transform.rotate(scaled_image, -90)
            # Blit the image onto the screen at a new position to avoid overlap
            screen.blit(rotated_image, (700, 50))
        except Exception as e:
            print(f"Error rendering image: {e}")



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
    screen_width, screen_height = 900, 900
    screen = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)
    pygame.display.set_caption("UART Control")
    font = pygame.font.Font(None, 36)

    running = True
    last_cmd = "None"
    def uart_thread():
        nonlocal running, last_cmd
        while running:
            last_cmd, tx_status = handle_uart_tx(uart)
            rx_status = handle_uart_rx(uart)
            running = tx_status and rx_status

    # Start the UART thread
    uart_thread_instance = threading.Thread(target=uart_thread, daemon=True)
    uart_thread_instance.start()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                handle_key_event(event, is_pressed=True)
            elif event.type == pygame.KEYUP:
                handle_key_event(event, is_pressed=False)

        draw_ui(screen, font, active_keys, last_cmd)

    pygame.quit()

if __name__ == "__main__":
    main()
    