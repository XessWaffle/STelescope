import pygame

from .ui_component import UIComponent
from uart_handler import OrientationDataLogPacket

class OrientationComponent(UIComponent):

    def __init__(self, screen: pygame.Surface, x, y, width, height):
        super().__init__(screen, x, y, width, height)

        self.orientation_data = None
        self.microstep_mode = 0

    def set_orientation_packet(self, orientation_data: OrientationDataLogPacket):
        self.orientation_data = orientation_data

    def set_microstep_mode(self, microstep_mode):
        self.microstep_mode = microstep_mode

    def draw(self):

        screen = self.screen
        font = pygame.font.Font(None, 36)
        orientation_packet = self.orientation_data
        microstep_mode = self.microstep_mode

        if orientation_packet == None:
            return

        bar_width = 200
        bar_height = 20
        base_x = self.x
        base_y = self.y
        max_steps = (360 / 1.8) * 1620 * (1 << microstep_mode)  # Assuming angles are in degrees
        spacing = 40  # Spacing between elements

        # Yaw
        yaw_width = int((orientation_packet["yaw_position"] / max_steps) * bar_width)
        pygame.draw.rect(screen, (255, 0, 0), (base_x, base_y, yaw_width, bar_height))
        yaw_text = font.render(f"Yaw: {orientation_packet['yaw_position']}", True, (255, 255, 255))
        screen.blit(yaw_text, (base_x + bar_width + 10, base_y))

        # Roll
        roll_width = int((orientation_packet["roll_position"] / max_steps) * bar_width)
        pygame.draw.rect(screen, (0, 255, 0), (base_x, base_y + spacing, roll_width, bar_height))
        roll_text = font.render(f"Roll: {orientation_packet['roll_position']}", True, (255, 255, 255))
        screen.blit(roll_text, (base_x + bar_width + 10, base_y + spacing))

        # Pitch
        pitch_width = int((orientation_packet["pitch_position"] / max_steps) * bar_width)
        pygame.draw.rect(screen, (0, 0, 255), (base_x, base_y + 2 * spacing, pitch_width, bar_height))
        pitch_text = font.render(f"Pitch: {orientation_packet['pitch_position']}", True, (255, 255, 255))
        screen.blit(pitch_text, (base_x + bar_width + 10, base_y + 2 * spacing))

        # Display gyroscope data as a radar-like visualization
        center_x, center_y = 150, 400
        radius = 100
        pygame.draw.circle(screen, (255, 255, 255), (center_x, center_y), radius, 1)
        pygame.draw.line(screen, (255, 0, 0), (center_x, center_y),
                 (center_x + int(orientation_packet["out_x_g_raw"] / 32768 * radius),
                  center_y - int(orientation_packet["out_y_g_raw"] / 32768 * radius)), 2)
        gyro_text = font.render("Gyro", True, (255, 255, 255))
        screen.blit(gyro_text, (center_x - 20, center_y + radius + 10))

        # Display accelerometer data as a radar-like visualization
        accel_center_y = center_y + 2 * radius + 50  # Adjusted to avoid overlap
        pygame.draw.circle(screen, (255, 255, 255), (center_x, accel_center_y), radius, 1)
        pygame.draw.line(screen, (0, 255, 0), (center_x, accel_center_y),
                 (center_x + int(orientation_packet["out_x_a_raw"] / 32768 * radius),
                  accel_center_y - int(orientation_packet["out_y_a_raw"] / 32768 * radius)), 2)
        accel_text = font.render("Accel", True, (255, 255, 255))
        screen.blit(accel_text, (center_x - 20, accel_center_y + radius + 10))

        # Display onboard magnetometer data as a radar-like visualization
        onboard_center_x = center_x + 300
        pygame.draw.circle(screen, (255, 255, 255), (onboard_center_x, center_y), radius, 1)
        pygame.draw.line(screen, (0, 0, 255), (onboard_center_x, center_y),
                 (onboard_center_x + int(orientation_packet["out_x_raw_onboard"] / 5000 * radius),
                  center_y - int(orientation_packet["out_y_raw_onboard"] / 5000 * radius)), 2)
        onboard_text = font.render("Onboard Mag", True, (255, 255, 255))
        screen.blit(onboard_text, (onboard_center_x - 20, center_y + radius + 10))

        # Display offboard magnetometer data as a radar-like visualization
        offboard_center_y = accel_center_y  # Align with accelerometer visualization
        pygame.draw.circle(screen, (255, 255, 255), (onboard_center_x, offboard_center_y), radius, 1)
        pygame.draw.line(screen, (255, 255, 0), (onboard_center_x, offboard_center_y),
                 (onboard_center_x + int(orientation_packet["out_x_raw_offboard"] / 5000 * radius),
                  offboard_center_y - int(orientation_packet["out_y_raw_offboard"] / 5000 * radius)), 2)
        offboard_text = font.render("Offboard Mag", True, (255, 255, 255))
        screen.blit(offboard_text, (onboard_center_x - 20, offboard_center_y + radius + 10))