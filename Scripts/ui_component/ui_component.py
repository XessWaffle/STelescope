import pygame
from abc import ABC, abstractmethod

class UIComponent(ABC):
    def __init__(self, screen: pygame.Surface, x, y, width, height):
        """
        Initialize the UIComponent.

        :param screen: The pygame screen object where the component will be drawn.
        :param x: The x-coordinate of the top-left corner of the component.
        :param y: The y-coordinate of the top-left corner of the component.
        :param width: The width of the component.
        :param height: The height of the component.
        """
        self.screen = screen
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    @abstractmethod
    def draw(self):
        """
        Abstract method to draw the component on the screen.
        Must be implemented by subclasses.
        """
        pass


class UIComponentManager:
    def __init__(self):
        """
        Initialize the UIComponentManager with an empty list of components.
        """
        self.components = []

    def add_component(self, component: UIComponent):
        """
        Add a UIComponent to the manager.

        :param component: The UIComponent to be added.
        """
        self.components.append(component)

    def remove_component(self, component: UIComponent):
        """
        Remove a UIComponent from the manager.

        :param component: The UIComponent to be removed.
        """
        if component in self.components:
            self.components.remove(component)

    def draw_components(self):
        """
        Iterate over all components and call their draw method.
        """
        for component in self.components:
            component.draw()

class UIManager:
    def __init__(self, title: str, screen_width: int, screen_height: int, component_manager: UIComponentManager = None):
        """
        Initialize the UIManager.

        :param title: The title of the pygame window.
        :param screen_width: The width of the pygame window.
        :param screen_height: The height of the pygame window.
        :param component_manager: The UIComponentManager to manage UI components.
        """
        pygame.init()
        self.screen = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)
        pygame.display.set_caption(title)

        if component_manager == None:
            self.component_manager = UIComponentManager()

    def get_component_manager(self):
        """ Get the current component manager """
        return self.component_manager
    
    def set_component_manager(self, component_manager: UIComponent):
        """ 
        Change the component manager
        
        :param component_manager: The new component manager
        """
        self.component_manager = component_manager

    def get_screen(self):
        return self.screen

    def draw(self):
        """
        Clear the screen, draw all components, and update the display.
        """
        self.screen.fill((0, 0, 0))  # Clear the screen with black color
        if(self.component_manager != None):
            self.component_manager.draw_components()
        pygame.display.flip()