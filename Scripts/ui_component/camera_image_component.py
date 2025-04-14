import pygame

from .ui_component import UIComponent
from uart_handler import CameraDataLogPacket

class CameraComponent(UIComponent):

    def __init__(self, screen: pygame.Surface, x, y, width, height):
        super().__init__(screen, x, y, width, height)

        self.image_raw = [0] * (320 * 240 * 3)

    def add_camera_data(self, cdlp: CameraDataLogPacket):

        idx = int((len(cdlp["camera_data"]) * cdlp["last_packet"] / 2) * 3)

        if(idx > len(self.image_raw) - 3):
            return
        # Convert each pair of bytes (RGB565) into three bytes (RGB888)
        for i in range(0, len(cdlp["camera_data"]), 2):
            if i + 1 < len(cdlp["camera_data"]) and idx < 320 * 240 * 3:
                rgb565 = (cdlp["camera_data"][i] << 8) | cdlp["camera_data"][i + 1]
                self.image_raw[idx + 0] = ((rgb565 >> 11) & 0x1F) * 255 // 31
                self.image_raw[idx + 1] = ((rgb565 >> 5) & 0x3F) * 255 // 63
                self.image_raw[idx + 2] = (rgb565 & 0x1F) * 255 // 31

                idx = idx + 3

    def draw(self):

        screen = self.screen

        base_x = self.x
        base_y = self.y

        image_data = bytes(self.image_raw)

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
                screen.blit(rotated_image, (base_x, base_y))
            except Exception as e:
                print(f"Error rendering image: {e}")
