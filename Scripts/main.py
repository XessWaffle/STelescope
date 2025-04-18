import pygame

from uart_handler import *
from ui_component import *
from telescope import *
import threading

active_keys = []
microstep_mode = 0

rx_mgr = UARTRxManager()
tx_mgr = UARTTxManager()
ui_mgr = UIManager("STM Telescope UI", 900, 900)

cmd_component = CommandComponent(ui_mgr.get_screen(), 10, 10, 0, 0)
orient_component = OrientationComponent(ui_mgr.get_screen(), 10, 100, 0, 0)
camera_component = CameraComponent(ui_mgr.get_screen(), 700, 100, 0, 0)

phy_telescope = PhysicalTelescope()

def handle_key_event(event, is_pressed):
    """Handle key press or release events."""
    key = event.key
    cmd = None
    global microstep_mode

    if is_pressed:
        if key in active_keys:
            return
        active_keys.append(key)

        match key:
            case pygame.K_q:
                cmd = UARTCommand(UARTCommands.CMD_R_PLUS, 0)
            case pygame.K_w:
                cmd = UARTCommand(UARTCommands.CMD_P_PLUS, 0)
            case pygame.K_e:
                cmd = UARTCommand(UARTCommands.CMD_R_MINUS, 0)
            case pygame.K_a:
                cmd = UARTCommand(UARTCommands.CMD_Y_PLUS, 0)
            case pygame.K_s:
                cmd = UARTCommand(UARTCommands.CMD_P_MINUS, 0)
            case pygame.K_d:
                cmd = UARTCommand(UARTCommands.CMD_Y_MINUS, 0)
            case pygame.K_h:
                cmd = UARTCommand(UARTCommands.CMD_HOME, 0)
            case pygame.K_m:
                cmd = UARTCommand(UARTCommands.CMD_MICST_MODE, microstep_mode)
                orient_component.set_microstep_mode(microstep_mode)
                microstep_mode = microstep_mode + 1
                microstep_mode = microstep_mode % 5
            case _:
                cmd = None
    else:
        if key in active_keys:
            active_keys.remove(key)

        match key:
            case pygame.K_q:
                cmd = UARTCommand(UARTCommands.CMD_R_STOP, 0)
            case pygame.K_w:
                cmd = UARTCommand(UARTCommands.CMD_P_STOP, 0)
            case pygame.K_e:
                cmd = UARTCommand(UARTCommands.CMD_R_STOP, 0)
            case pygame.K_a:
                cmd = UARTCommand(UARTCommands.CMD_Y_STOP, 0)
            case pygame.K_s:
                cmd = UARTCommand(UARTCommands.CMD_P_STOP, 0)
            case pygame.K_d:
                cmd = UARTCommand(UARTCommands.CMD_Y_STOP, 0)
            case _:
                cmd = None

        cmd_component.set_active_keys(active_keys)
        cmd_component.set_command(cmd)

    if cmd is not None:
        tx_mgr.write_command(cmd)

def main():
    # Start listening to keyboard events
    running = True

    ui_component_manager = ui_mgr.get_component_manager()
    ui_component_manager.add_component(cmd_component)
    ui_component_manager.add_component(orient_component)
    ui_component_manager.add_component(camera_component)

    def uart_processing_thread():
        data_packet = None
        type = None
        while running:
            rx_mgr.check_and_process_uart()

            if data_packet == None:
                type, data_packet = rx_mgr.poll_packet()
                continue
            
            if(not data_packet.is_ready()):
                continue

            if type == OrientationDataLogPacket:
                orient_component.set_orientation_packet(data_packet)
                phy_telescope.orientation_update(data_packet)
            elif type == CameraDataLogPacket:
                camera_component.add_camera_data(data_packet)

            data_packet = None

    uart_thread = threading.Thread(target=uart_processing_thread, daemon=True)
    uart_thread.start()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                handle_key_event(event, is_pressed=True)
            elif event.type == pygame.KEYUP:
                handle_key_event(event, is_pressed=False)

        ui_mgr.draw()

    pygame.quit()

if __name__ == "__main__":
    main()