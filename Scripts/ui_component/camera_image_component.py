import pygame

from .ui_component import UIComponent
from uart_handler import CameraDataLogPacket
import io
from PIL import Image
from PIL import ImageEnhance

from queue import Queue
import threading
import os


class CameraComponent(UIComponent):

    def __init__(self, screen: pygame.Surface, x, y, width, height):
        super().__init__(screen, x, y, width, height)
        
        self.jpeg_bytes = io.BytesIO()

        self.ready_buffs = Queue()
        self.ready_lock = threading.Lock()

        self.processing_queue = Queue()
        self.process_lock = threading.Lock()

        self._capturing_header = False
        self._prev_byte = 0

        self._cached_image_surface = None
        self._jpegs_found = 0

        # Run the processing in a separate thread
        # Add the processing function to a queue and process it in a separate thread
        def process_queue():
            while True:
                jpeg_proc_task = None
                cdlp = None

                self.process_lock.acquire()
                if(self.processing_queue.qsize() > 0):
                    jpeg_proc_task, cdlp = self.processing_queue.get()
                self.process_lock.release()

                if jpeg_proc_task is None:  # Exit condition for the thread
                    continue
                jpeg_proc_task(cdlp)

        # Create a queue for tasks
        threading.Thread(target=process_queue, daemon=True).start()

    def _switch_jpeg_buff(self):
        self.jpeg_bytes = io.BytesIO()

    # Append the camera data bytes to the jpeg_bytes buffer
    def _process_camera_data(self, cdlp):
        for byte in cdlp["camera_data"]:
            if byte == 0xE0 and self._prev_byte == 0xFF:
                self.jpeg_bytes.write((0xFF).to_bytes())
                self.jpeg_bytes.write((0xD8).to_bytes())
                self.jpeg_bytes.write(self._prev_byte.to_bytes())
                self._capturing_header = True

            if byte == 0xD9 and self._prev_byte == 0xFF and self._capturing_header:
                self.jpeg_bytes.write(byte.to_bytes())

                self.ready_lock.acquire()
                self.ready_buffs.put(self.jpeg_bytes)
                self.ready_lock.release()

                self._switch_jpeg_buff()

                self._capturing_header = False
                self._jpegs_found = self._jpegs_found + 1

            if self._capturing_header:
                self.jpeg_bytes.write(byte.to_bytes())

            self._prev_byte = byte

    def add_camera_data(self, cdlp: CameraDataLogPacket):

        """
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
        """

        self.process_lock.acquire()
        self.processing_queue.put((self._process_camera_data, cdlp))
        self.process_lock.release()

            

    def draw(self):

        screen = self.screen

        base_x = self.x
        base_y = self.y
        
        use_cache = False
        next_jpeg = None

        self.ready_lock.acquire()

        if self.ready_buffs.qsize() == 0:
            use_cache = True

        if not use_cache:
            next_jpeg = self.ready_buffs.get()

        self.ready_lock.release()

        if use_cache:
            if self._cached_image_surface is not None:
                screen.blit(self._cached_image_surface, (base_x, base_y))
            
            return

        # Decode the JPEG image data
        try:

            # Convert the JPEG bytes to a PIL image
            pil_image = Image.open(next_jpeg)
            pil_image = pil_image.convert("RGB")  # Ensure it's in RGB format
            
            # Resize the image
            pil_image = pil_image.resize((1024, 768))

            # Convert the PIL image to a pygame surface
            image_surface = pygame.image.fromstring(
                pil_image.tobytes(), pil_image.size, pil_image.mode
            )

            # Rotate the image 90 degrees
            #rotated_image = pygame.transform.rotate(image_surface, -90)

            # Blit the image onto the screen
            # Create a circular mask
            radius = min(image_surface.get_width(), image_surface.get_height())
            mask = pygame.Surface(image_surface.get_size(), pygame.SRCALPHA)
            mask.fill((0, 0, 0, 0))
            pygame.draw.ellipse(
                mask,
                (255, 255, 255, 255),
                (0, 0, radius, radius),
            )

            # Apply the mask to the image surface
            circular_image = image_surface.copy()
            circular_image.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)

            # Blit the circular image onto the screen
            screen.blit(circular_image, (base_x, base_y))

            self._cached_image_surface = circular_image
        except Exception as e:
            if self._cached_image_surface is not None:
                screen.blit(self._cached_image_surface, (base_x, base_y))

