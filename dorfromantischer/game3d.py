"""Base classes for implementing trivial `pygame`s."""
from datetime import datetime
import sys

import pygame
from OpenGL import GL
from typing import cast

class Game3D:
    """Base 3D game class."""

    WIDTH = 1400
    HEIGHT = 1000

    _fps = 60

    avg_ms_per_frame: float

    def __init__(self):
        """Initialize a window."""
        pygame.init()

        # This breaks text?!
        # pygame.display.gl_set_attribute(pygame.GL_CONTEXT_MAJOR_VERSION, 4)
        # pygame.display.gl_set_attribute(pygame.GL_CONTEXT_MINOR_VERSION, 1)
        # pygame.display.gl_set_attribute(pygame.GL_CONTEXT_PROFILE_MASK, pygame.GL_CONTEXT_PROFILE_CORE)

        self.window_size = self.WIDTH, self.HEIGHT
        self.screen = pygame.display.set_mode(self.window_size, pygame.DOUBLEBUF|pygame.OPENGL|pygame.HWSURFACE)

        GL.glViewport(0, 0, self.WIDTH, self.HEIGHT)
        GL.glClearColor(0.5, 0, 0.5, 0)

        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)

    def reset(self):
        """Override this to easily reset the game state from the main."""
        pass

    def handle_event(self, event):
        """Handle additional events."""
        pass

    def update(self):
        """Override this function to update global state in each game tick."""
        pass

    def render(self):
        """Override this function to render global stuff in each game tick."""
        pass

    def drop(self):
        """Override this function to delete/close resources when game terminates."""
        pass

    def current_time(self) -> float:
        return pygame.time.get_ticks() / 1000

    def log(self, string: str):
        time = datetime.now().time()
        print(f"[{time.strftime('%H:%M:%S')}] {string}")

    def run(self):
        """Run the game."""
        self._running = True
        clock = pygame.time.Clock()

        self.avg_ms_per_frame = 1000 / self._fps

        while self._running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()
                elif event.type == pygame.KEYDOWN:
                    if event.unicode == "q":
                        sys.exit()
                self.handle_event(event)

            self.update()

            GL.glClear(cast(GL.GLenum, GL.GL_COLOR_BUFFER_BIT) | cast(GL.GLenum, GL.GL_DEPTH_BUFFER_BIT))
            self.render()
            pygame.display.flip()

            ms_per_frame = clock.tick(self._fps)
            self.avg_ms_per_frame = 0.95 * self.avg_ms_per_frame + 0.05 * ms_per_frame

        self.drop()

#
#
# def load_texture(filename):
#     img = Image.open(filename, 'r').convert("RGB")
#     img_data = np.array(img, dtype=np.uint8)
#     w, h = img.size
#
#     texture = glGenTextures(1)
#
#     glBindTexture(GL_TEXTURE_2D, texture)
#
#     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
#     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
#
#     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data)
#
#     return texture
#
#
# def perspective(fovy, aspect, z_near, z_far):
#     f = 1 / math.tan(math.radians(fovy) / 2)
#     return np.array([
#         [f / aspect,  0,                                   0,  0],
#         [          0, f,                                   0,  0],
#         [          0, 0, (z_far + z_near) / (z_near - z_far), -1],
#         [          0, 0, (2*z_far*z_near) / (z_near - z_far),  0]
#     ])
#
# def rotate(angle, x, y, z):
#     s = math.sin(math.radians(angle))
#     c = math.cos(math.radians(angle))
#     magnitude = math.sqrt(x*x + y*y + z*z)
#     nc = 1 - c
#
#     x /= magnitude
#     y /= magnitude
#     z /= magnitude
#
#     return np.array([
#         [     c + x**2 * nc, y * x * nc - z * s, z * x * nc + y * s, 0],
#         [y * x * nc + z * s,      c + y**2 * nc, y * z * nc - x * s, 0],
#         [z * x * nc - y * s, z * y * nc + x * s,      c + z**2 * nc, 0],
#         [                 0,                  0,                  0, 1],
#     ])
#
# _vertices = [
#     ( 1.000000, -1.000000, -1.000000),
#     ( 1.000000, -1.000000,  1.000000),
#     (-1.000000, -1.000000,  1.000000),
#     (-1.000000, -1.000000, -1.000000),
#     ( 1.000000,  1.000000, -0.999999),
#     ( 0.999999,  1.000000,  1.000001),
#     (-1.000000,  1.000000,  1.000000),
#     (-1.000000,  1.000000, -1.000000),
# ]
# _normals = [
#     ( 0.000000, -1.000000,  0.000000),
#     ( 0.000000,  1.000000,  0.000000),
#     ( 1.000000,  0.000000,  0.000000),
#     (-0.000000,  0.000000,  1.000000),
#     (-1.000000, -0.000000, -0.000000),
#     ( 0.000000,  0.000000, -1.000000),
# ]
#
#
# _texcoords = [
#     (0.250043, 0.749957),
#     (0.250043, 0.500000),
#     (0.500000, 0.500000),
#     (0.500000, 0.250043),
#     (0.250043, 0.250043),
#     (0.250044, 0.000087),
#     (0.500000, 0.999913),
#     (0.250043, 0.999913),
#     (0.000087, 0.749956),
#     (0.000087, 0.500000),
#     (0.500000, 0.749957),
#     (0.749957, 0.500000),
#     (0.500000, 0.000087),
#     (0.749957, 0.749957),
# ]
# _vertex_triangles = [
#     (1, 2, 3),
#     (7, 6, 5),
#     (4, 5, 1),
#     (5, 6, 2),
#     (2, 6, 7),
#     (0, 3, 7),
#     (0, 1, 3),
#     (4, 7, 5),
#     (0, 4, 1),
#     (1, 5, 2),
#     (3, 2, 7),
#     (4, 0, 7),
# ]
#
# _texture_triangles = [
#     ( 0,  1,  2),
#     ( 3,  4,  5),
#     ( 6,  7,  0),
#     ( 8,  9,  1),
#     ( 1,  4,  3),
#     (10,  2, 11),
#     (10,  0,  2),
#     (12,  3,  5),
#     (10,  6,  0),
#     ( 0,  8,  1),
#     ( 2,  1,  3),
#     (13, 10, 11),
# ]
#
# _normal_triangles = [
#     (0, 0, 0),
#     (1, 1, 1),
#     (2, 2, 2),
#     (3, 3, 3),
#     (4, 4, 4),
#     (5, 5, 5),
#     (0, 0, 0),
#     (1, 1, 1),
#     (2, 2, 2),
#     (3, 3, 3),
#     (4, 4, 4),
#     (5, 5, 5),
# ]
#
# vertices = np.array([
#     _vertices[index]
#     for indices in _vertex_triangles
#     for index in indices
# ])
#
# normals = np.array([
#     _normals[index]
#     for indices in _normal_triangles
#     for index in indices
# ])
#
# texcoords = np.array([
#     _texcoords[index]
#     for indices in _texture_triangles
#     for index in indices
# ])
#
#
# if __name__ == "__main__":
#
#     uMVMatrix = glGetUniformLocation(program, "uMVMatrix")
#     uPMatrix = glGetUniformLocation(program, "uPMatrix")
#     sTexture = glGetUniformLocation(program, "sTexture")
#
#     aVertex = glGetAttribLocation(program, "aVertex")
#     aNormal = glGetAttribLocation(program, "aNormal")
#     aTexCoord = glGetAttribLocation(program, "aTexCoord")
#
#     glUseProgram(program)
#     glEnableVertexAttribArray(aVertex)
#     glEnableVertexAttribArray(aNormal)
#     glEnableVertexAttribArray(aTexCoord)
#
#     texture = load_texture("texture.png")
#
#     glActiveTexture(GL_TEXTURE0);
#     glBindTexture(GL_TEXTURE_2D, texture)
#     glUniform1i(sTexture, 0)
#
#     glEnable(GL_DEPTH_TEST)
#
#     running = True
#     while running:
#
#         # model_matrix = np.dot(model_matrix, rotate(1, 1, 0.5, 0))
#
#         glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
#
#         glVertexAttribPointer(aVertex, 3, GL_FLOAT, GL_FALSE, 0, vertices)
#         glVertexAttribPointer(aNormal, 3, GL_FLOAT, GL_FALSE, 0, normals)
#         glVertexAttribPointer(aTexCoord, 2, GL_FLOAT, GL_FALSE, 0, texcoords)
#
#         mv_matrix = np.dot(model_matrix, view_matrix)
#         glUniformMatrix4fv(uMVMatrix, 1, GL_FALSE, mv_matrix)
#         glUniformMatrix4fv(uPMatrix, 1, GL_FALSE, projection_matrix)
#
#         glDrawArrays(GL_TRIANGLES, 0, len(vertices))
#
#
#         pygame.display.flip()
#
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT:
#                 running = False
#             if event.type == pygame.MOUSEMOTION:
#                 x, y = event.rel
#                 if any(event.buttons):
#                     model_matrix = model_matrix.dot(rotate(y, -1, 0, 0)).dot(rotate(x, 0, -1, 0))
