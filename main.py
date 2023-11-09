# https://stackoverflow.com/questions/3052202/how-to-analyse-contents-of-binary-serialization-stream/30176566#30176566

import sys
from dorfromantischer.savegame import parse_savegame
from dorfromantischer.state import State
from dorfromantischer.game3d import Game3D
from dorfromantischer.gl.program import Program
from dorfromantischer.gl.buffer import Buffer, UniformBuffer, ShaderStorageBuffer
import pygame
import math
from OpenGL import GL
import numpy
from scipy.spatial.transform import Rotation
from typing import cast, Optional


def normalize(vector: numpy.ndarray):
    return vector / numpy.linalg.norm(vector)


# Byte sizes.
bool_ = 4
int_ = 4
float_ = 4
vec3_ = (3 + 1) * float_



class Texture:
    texture: GL.GLuint
    bound_index: int

    def __init__(self, w: int, h: int, num_components: GL.GLint, format: Optional[GL.GLint]=None):
        self.texture = GL.glGenTextures(1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.texture)

        GL.glTexImage2D(
            GL.GL_TEXTURE_2D,
            0,
            num_components,
            w,
            h,
            0,
            num_components if format is None else format,
            GL.GL_FLOAT,
            None
        )

        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_CLAMP_TO_EDGE)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_CLAMP_TO_EDGE)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        GL.glBindTexture(GL.GL_TEXTURE_2D, 0)

    def bind(self, bind_index: int):
        self.bound_index = bind_index
        GL.glActiveTexture(cast(GL.GLenum, GL.GL_TEXTURE0) + self.bound_index)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.texture)

    def unbind(self):
        GL.glActiveTexture(cast(GL.GLenum, GL.GL_TEXTURE0) + self.bound_index)
        GL.glBindTexture(GL.GL_TEXTURE_2D, 0)
        self.bound_index = 0

    def drop(self):
        GL.glDeleteTextures([self.texture])


class Framebuffer:
    textures: list[Texture]
    framebuffer: GL.GLuint

    def __init__(self, textures: list[Texture]):
        self.textures = textures
        self.framebuffer = GL.glGenFramebuffers(1)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self.framebuffer)

        base_enum = cast(int, GL.GL_COLOR_ATTACHMENT0)
        for index, texture in enumerate(textures):
            GL.glFramebufferTexture2D(GL.GL_FRAMEBUFFER, base_enum + index, GL.GL_TEXTURE_2D, texture.texture, 0)
        GL.glDrawBuffers([base_enum + i for i in range(len(textures))])

        if GL.glCheckFramebufferStatus(GL.GL_FRAMEBUFFER) != GL.GL_FRAMEBUFFER_COMPLETE:
            print("Something wrong with framebuffer...")

        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)

    def bind(self):
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self.framebuffer)

    def unbind(self):
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)

    def drop(self):
        GL.glDeleteFramebuffers([self.framebuffer])


class Sampler:
    sampler: GL.GLuint

    def __init__(self):
        self.sampler = GL.glGenSamplers(1)
        GL.glSamplerParameteri(self.sampler, GL.GL_TEXTURE_WRAP_S, GL.GL_REPEAT)
        GL.glSamplerParameteri(self.sampler, GL.GL_TEXTURE_WRAP_T, GL.GL_REPEAT)
        GL.glSamplerParameteri(self.sampler, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        GL.glSamplerParameteri(self.sampler, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)

    def bind(self, bind_index: int):
        GL.glBindSampler(bind_index, self.sampler)

    def drop(self):
        GL.glDeleteSamplers([self.sampler])


class Dorfromantik(Game3D):
    state: State

    mouse_locked: bool

    yaw: float
    pitch: float

    origin: numpy.ndarray
    right: numpy.ndarray
    ahead: numpy.ndarray
    up: numpy.ndarray

    shader: Program
    lighting: Program

    globals_buffer: Buffer
    view_buffer: Buffer
    quadrant_buffers: list[Buffer]

    font: pygame.font.Font

    fps: int
    fps_surface: pygame.Surface
    fps_data: bytes

    framebuffer: Framebuffer
    data: Texture
    sampler: Sampler

    def __init__(self):
        super().__init__()

        self.shader = Program("shaders/quad.vert", "shaders/shader.frag")
        self.lighting = Program("shaders/quad.vert", "shaders/lighting.frag")
        self.font = pygame.font.SysFont('arial', 16)
        self.fps = 0

        self.data = Texture(self.WIDTH, self.HEIGHT, GL.GL_RGBA32F, GL.GL_RGBA)
        self.framebuffer = Framebuffer([self.data])
        self.sampler = Sampler()

        self.vao = GL.glGenVertexArrays(1)

    def create_buffers(self):
        self.log("Create buffers")
        state = self.state

        # Globals buffer.
        globals_bytes = 1 * float_ + 3 * float_ + 4 * int_
        self.globals_buffer = UniformBuffer(globals_bytes, "globals_buffer")

        # View buffer.
        view_bytes = 2 * int_ + 2 * float_ + 4 * vec3_
        self.view_buffer = UniformBuffer(view_bytes, "view_buffer")

        # Quadrant buffers.
        tile_bytes = bool_ + int_ + 18 * int_ + 4 * int_
        self.quadrant_buffers = [
            ShaderStorageBuffer(tile_bytes * len(state.quadrants[q]), f"quadrant{q}_buffer")
            for q in range(4)
        ]

    def initialize_buffers(self):
        self.log("Initialize buffers")
        state = self.state
        quadrants = state.quadrants

        # View data
        view_data = numpy.array([self.WIDTH, self.HEIGHT], dtype=numpy.int32)
        self.view_buffer.write(view_data)
        view_data = numpy.array([math.pi / 2, 0], dtype=numpy.float32)
        self.view_buffer.write(view_data, offset=2 * int_)

        for q in range(4):
            quadrant = quadrants[q]
            tiles_data = numpy.zeros(shape=(len(quadrant), 1 + 1 + 18 + 4), dtype=numpy.int32)

            for tile in quadrant:
                segments = [0] * 18
                if tile is None:
                    continue

                for index, segment in enumerate(tile.segments):
                    segments[3 * index + 0] = segment.form.value
                    segments[3 * index + 1] = segment.terrain.value
                    segments[3 * index + 2] = segment.rotation

                for index in range(len(tile.segments), 6):
                    segments[3 * index + 0] = 0

                tiles_data[tile.index] = [1, tile.special_tile.id, *segments, 0, 0, 0, 0]

            self.quadrant_buffers[q].write(tiles_data)

    def rebind_buffers(self):
        self.log("Rebinding")
        program = self.shader
        if not program.program:
            self.log("Invalid shader, not rebinding buffers")
            return

        self.globals_buffer.rebind(program)
        self.view_buffer.rebind(program)
        for q in range(4):
            self.quadrant_buffers[q].rebind(program)

    def set_yaw_pitch(self, yaw, pitch):
        self.yaw = yaw
        self.pitch = pitch

        ahead = numpy.array([1, 0, 0])
        right = numpy.array([0, 0, 1])

        yaw_rotation = Rotation.from_rotvec(yaw * numpy.array([0, 1, 0]))
        ahead = yaw_rotation.apply(ahead)
        self.right = yaw_rotation.apply(right)

        pitch_rotation = Rotation.from_rotvec(pitch * self.right)
        self.ahead = pitch_rotation.apply(ahead)

        self.up = numpy.cross(self.right, self.ahead)

    def handle_event(self, event):
        if event.type == pygame.MOUSEMOTION:
            if self.mouse_locked:
                right, down = event.rel

                pi_2 = math.pi / 2
                yaw = self.yaw - right / 1000
                pitch = max(-pi_2, min(self.pitch - down / 1000, pi_2))

                self.set_yaw_pitch(yaw, pitch)

        elif event.type == pygame.KEYDOWN:
            if event.unicode == "m":
                self.lock_mouse(False)

        elif event.type == pygame.MOUSEBUTTONDOWN:
            self.lock_mouse(True)

    def lock_mouse(self, lock: bool):
        self.mouse_locked = lock
        pygame.mouse.set_visible(not lock)
        pygame.event.set_grab(lock)

    def set_data(self, state: State):
        self.state = state

        self.create_buffers()
        self.rebind_buffers()
        self.globals_buffer.rebind(self.lighting)
        self.view_buffer.rebind(self.lighting)
        self.initialize_buffers()
        self.reset()

    def write_globals_buffer(self):
        globals_data = numpy.array([self.current_time()], dtype=numpy.float32)
        self.globals_buffer.write(globals_data)
        globals_data = numpy.array([len(self.state.quadrants[i]) for i in range(4)], dtype=numpy.int32)
        self.globals_buffer.write(globals_data, offset=1 * float_ + 3 * int_)

    def write_view_buffer(self):
        view_data = numpy.array([*self.origin, 0, *self.right, 0, *self.ahead, 0, *self.up, 0], dtype=numpy.float32)
        self.view_buffer.write(view_data, offset=2 * int_ + 2 * float_)

    def reset(self):
        # self.origin = numpy.array([100.0, 0.0, 100.0])
        # self.set_yaw_pitch(7 * math.pi / 8, 0)

        # self.origin = numpy.array([0.0, 100.0, 0.0])
        # self.set_yaw_pitch(0, -math.pi / 2)

        self.origin = numpy.array([0.0, 5.0, 0.0])
        self.set_yaw_pitch(- math.pi / 4, 0)

        self.lock_mouse(False)

    def update(self):
        pressed = pygame.key.get_pressed()

        movement = numpy.array([0.0, 0.0, 0.0])
        movement += self.ahead * (int(pressed[pygame.K_w]) - int(pressed[pygame.K_s]))
        movement += self.right * (int(pressed[pygame.K_d]) - int(pressed[pygame.K_a]))
        movement_sqr = numpy.dot(movement, movement)
        if movement_sqr > 0.5:
            movement = movement / math.sqrt(movement_sqr) * 0.1
            factor = 10 if pressed[pygame.K_LSHIFT] else (0.1 if pressed[pygame.K_LCTRL] else 1)

            self.origin += factor * movement
            self.origin[1] = max(1.0, self.origin[1])

    def render(self):
        if self.shader.was_source_modified_since():
            self.log("Recompiling shader")
            if self.shader.reinstall_if_valid():
                self.rebind_buffers()

        if self.lighting.was_source_modified_since():
            self.log("Recompiling lighting shader")
            if self.lighting.reinstall_if_valid():
                self.globals_buffer.rebind(self.lighting)
                self.view_buffer.rebind(self.lighting)

        self.write_globals_buffer()
        self.write_view_buffer()

        GL.glBindVertexArray(self.vao)

        # Run regular pass.
        if self.shader.program:
            self.framebuffer.bind()
            self.shader.use()
            GL.glDrawArrays(GL.GL_TRIANGLE_STRIP, 0, 4)
            self.shader.unuse()
            self.framebuffer.unbind()

        # Run lighting pass.
        if self.lighting.program:
            # Bind actual textures to texture units.
            self.data.bind(0)
            self.sampler.bind(0)

            self.lighting.use()
            GL.glDrawArrays(GL.GL_TRIANGLE_STRIP, 0, 4)
            self.lighting.unuse()

            # Unbind textures.
            self.data.unbind()

        GL.glBindVertexArray(0)

        fps = int(1000 / self.avg_ms_per_frame)
        if fps != self.fps:
            self.fps = fps
            self.fps_surface = self.font.render(f"FPS: {self.fps}", True, (100, 100, 100, 255)).convert_alpha()
            self.fps_data = pygame.image.tostring(self.fps_surface, "RGBA", True)

        GL.glWindowPos2d(self.WIDTH - self.fps_surface.get_width(), self.HEIGHT - self.fps_surface.get_height())
        GL.glDrawPixels(self.fps_surface.get_width(), self.fps_surface.get_height(), GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, self.fps_data)

    def drop(self):
        self.sampler.drop()
        self.framebuffer.drop()
        self.data.drop()


if __name__ == "__main__":
    print("Loading GUI")
    game = Dorfromantik()

    print("Parsing savegame")
    data = parse_savegame(sys.argv[1:])

    print("Interpreting data")
    state = State.from_dump(data)
    state.assign_groups()

    print("Running game")
    game.set_data(state)
    game.reset()
    game.run()
