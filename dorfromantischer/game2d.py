"""Base classes for implementing trivial `pygame`s."""
import functools
import sys

import pygame


class Game2D:
    """Base game class.

    Wraps basic game routines and provides an overridable interface.
    """

    WIDTH = 1000
    HEIGHT = 1000

    _graphics = True
    _text = True
    _fps = 60

    @staticmethod
    def is_in_rect(x, y, rx, ry, rw, rh):
        """Check if a point `(x, y)` is in the given rect."""
        return x >= rx and x <= rx + rw and y >= ry and y <= ry + rh

    @staticmethod
    def is_on_screen(x, y):
        """Check if the point is visible on the screen."""
        return Game.is_in_rect(x, y, 0, 0, Game.WIDTH, Game.HEIGHT)

    def __init__(self):
        """Initialize a window."""
        pygame.init()
        self.font = pygame.font.SysFont("dejavusansmono", 12)

        self.window_size = self.WIDTH, self.HEIGHT
        self.full_rect = 0, 0, self.WIDTH, self.HEIGHT
        self.screen = pygame.display.set_mode(self.window_size)
        self.black = pygame.Color(0, 0, 0, 255)

    def reset(self):
        """Override this to easily reset the game state from the main."""
        pass

    def render_text(self, lines, pos):
        """Display text split by lines at position `pos`."""
        if not self._text:
            return

        lines = [(text, color, self.font.size(text)) for text, color in lines]
        text_width, text_height = functools.reduce(
            lambda total, line: (max(total[0], line[2][0]), total[1] + line[2][1]), lines, (0, 0)
        )

        for text, color, (_, height) in lines:
            text_surface = self.font.render(text, True, color)
            self.screen.blit(text_surface, dest=pos)
            pos = pos[0], pos[1] + height

    def handle_event(self, event):
        """Handle additional events."""
        pass

    def update(self):
        """Override this function to update global state in each game tick."""
        pass

    def render(self):
        """Override this function to render global stuff in each game tick."""
        pass

    def run(self):
        """Run the game."""
        self._running = True
        clock = pygame.time.Clock()

        while self._running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()
                elif event.type == pygame.KEYDOWN:
                    if event.unicode == "q":
                        sys.exit()
                    elif event.unicode == "t":
                        self._text = not self._text
                    elif event.unicode == "n":
                        self._running = False
                    elif event.unicode == "v":
                        self._graphics = not self._graphics
                    elif event.unicode == "-":
                        self._fps = max(1, self._fps - 10)
                    elif event.unicode == "+":
                        self._fps = min(self._fps + 10, 300)
                self.handle_event(event)

            self.update()

            if self._graphics:
                self.screen.fill(self.black, self.full_rect)
                self.render()
                pygame.display.flip()

                if self._fps is not None and self._fps < 300:
                    clock.tick(self._fps)
