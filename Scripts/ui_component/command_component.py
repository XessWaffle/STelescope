import pygame

from .ui_component import UIComponent
from uart_handler import UARTCommand


class CommandComponent(UIComponent):

    def __init__(self, screen: pygame.Surface, x, y, width, height):
        super().__init__(screen, x, y, width, height)

        self.command = None
        self.active_keys = []

    def set_command(self, uart_cmd: UARTCommand):
        self.command = uart_cmd
    
    def set_active_keys(self, active_keys):
        self.active_keys = active_keys

    def draw(self):

        screen = self.screen
        font = pygame.font.Font(None, 36)
        active_keys = self.active_keys
        last_cmd = None

        if self.command != None:
            last_cmd = self.command.to_bytes()

        base_x = self.x
        base_y = self.y

        # Display active keys
        active_keys_text = f"Active Keys: {', '.join([pygame.key.name(k) for k in active_keys])}"
        active_keys_surface = font.render(active_keys_text, True, (255, 255, 255))
        screen.blit(active_keys_surface, (base_x, base_y))

        # Display last command
        last_cmd_text = f"Last Command: {last_cmd}"
        last_cmd_surface = font.render(last_cmd_text, True, (255, 255, 255))
        screen.blit(last_cmd_surface, (base_x, base_y + 40))
