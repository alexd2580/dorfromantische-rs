# https://stackoverflow.com/questions/3052202/how-to-analyse-contents-of-binary-serialization-stream/30176566#30176566

import sys
from dorfromantischer.savegame import parse_savegame
from dorfromantischer.state import State, Form, Terrain
from dorfromantischer.chunks import Chunks
from dorfromantischer.game3d import Game3D
from dorfromantischer.gl.program import Program
from dorfromantischer.gl.buffer import Buffer, UniformBuffer, ShaderStorageBuffer
import pygame
import math
from OpenGL import GL
import numpy
from scipy.spatial.transform import Rotation
from typing import Optional


def normalize(vector: numpy.ndarray):
    return vector / numpy.linalg.norm(vector)



class Dorfromantik(Game3D):
    state: State
    chunks: Chunks

    mouse_locked: bool

    yaw: float
    pitch: float

    origin: numpy.ndarray
    right: numpy.ndarray
    ahead: numpy.ndarray
    up: numpy.ndarray

    shader: Program

    globals_buffer: Buffer
    view_buffer: Buffer
    chunk_meta_buffer: Buffer
    tiles_buffer: Buffer
    chunk_headers_buffer: Buffer
    chunk_indices_buffer: Buffer

    def __init__(self):
        super().__init__()

        self.shader = Program("shaders/vertex.glsl", "shaders/fragment.glsl")

    def create_buffers(self):
        self.log("Create buffers")
        state = self.state
        tiles = state.tiles
        chunks = self.chunks

        # Globals buffer
        globals_bytes = 4
        self.globals_buffer = UniformBuffer(globals_bytes, "globals_buffer")

        # View buffer
        view_bytes = 2 * 4 + 4 * 4 + 4 + 4 + 4 * (3 + 1) * 4
        self.view_buffer = UniformBuffer(view_bytes, "view_buffer")
        # Chunk meta buffer
        chunk_meta_bytes = 4 * 4
        self.chunk_meta_buffer = UniformBuffer(chunk_meta_bytes, "chunk_meta_buffer")

        # Tiles buffer
        num_tile_4bytes = 2 + 6
        tile_bytes = num_tile_4bytes * 4
        tiles_bytes = len(tiles) * tile_bytes
        self.tiles_buffer = ShaderStorageBuffer(tiles_bytes, "tiles_buffer")

        # Chunk headers buffer
        chunk_header_bytes = 2 * 4
        chunk_headers_bytes = len(chunks.headers) * chunk_header_bytes
        self.chunk_headers_buffer = ShaderStorageBuffer(chunk_headers_bytes, "chunk_headers_buffer")

        # Chunk indices buffer
        chunk_index_bytes = 4
        chunk_indices_bytes = len(chunks.headers) * chunks.chunk_size * chunks.chunk_size * chunk_index_bytes
        self.chunk_indices_buffer = ShaderStorageBuffer(chunk_indices_bytes, "chunk_indices_buffer")

    def initialize_buffers(self):
        self.log("Initialize buffers")
        state = self.state
        tiles = state.tiles
        chunks = self.chunks

        view_data = numpy.array([self.WIDTH, self.HEIGHT], dtype=numpy.int32)
        self.view_buffer.write(view_data)


        chunk_size = chunks.chunk_size
        chunk_meta_data = numpy.array([chunk_size, chunk_size * chunk_size, chunks.num_chunk_levels, chunks.num_level0_chunks], dtype=numpy.int32)
        self.chunk_meta_buffer.write(chunk_meta_data)

        terrains = set()
        forms = set()
        special = set()

        def rotation(rot):
            return rot % 6

        tiles_data = numpy.zeros((2 + 6) * len(tiles), dtype=numpy.int32)
        for index, tile in enumerate(tiles):
            segments = [Terrain.empty.value] * 6

            for segment in tile.segments:
                rot = rotation(tile.rotation + segment.rotation)
                terrain = segment.terrain.value
                match segment.form:
                    case Form.size1:
                        segments[rotation(rot + 0)] = terrain
                    case Form.size2:
                        segments[rotation(rot + 0)] = terrain
                        segments[rotation(rot + 1)] = terrain
                    case Form.bridge:
                        segments[rotation(rot + 0)] = terrain
                        segments[rotation(rot + 2)] = terrain
                    case Form.straight:
                        segments[rotation(rot + 0)] = terrain
                        segments[rotation(rot + 3)] = terrain
                    case Form.size3:
                        segments[rotation(rot + 0)] = terrain
                        segments[rotation(rot + 1)] = terrain
                        segments[rotation(rot + 2)] = terrain
                    case Form.junction_left:
                        segments[rotation(rot + 0)] = terrain
                        segments[rotation(rot + 1)] = terrain
                        segments[rotation(rot + 3)] = terrain
                    case Form.junction_right:
                        segments[rotation(rot + 0)] = terrain
                        segments[rotation(rot + 1)] = terrain
                        segments[rotation(rot + 4)] = terrain
                    case Form.three_way:
                        segments[rotation(rot + 0)] = terrain
                        segments[rotation(rot + 2)] = terrain
                        segments[rotation(rot + 4)] = terrain
                    case Form.size4:
                        segments[rotation(rot + 0)] = terrain
                        segments[rotation(rot + 1)] = terrain
                        segments[rotation(rot + 2)] = terrain
                        segments[rotation(rot + 3)] = terrain
                    case Form.fan_out:
                        segments[rotation(rot + 0)] = terrain
                        segments[rotation(rot + 1)] = terrain
                        segments[rotation(rot + 2)] = terrain
                        segments[rotation(rot + 4)] = terrain
                    case Form.x:
                        segments[rotation(rot + 0)] = terrain
                        segments[rotation(rot + 1)] = terrain
                        segments[rotation(rot + 3)] = terrain
                        segments[rotation(rot + 4)] = terrain
                    case Form.size5:
                        segments[rotation(rot + 0)] = terrain
                        segments[rotation(rot + 1)] = terrain
                        segments[rotation(rot + 2)] = terrain
                        segments[rotation(rot + 3)] = terrain
                        segments[rotation(rot + 4)] = terrain
                    case Form.size6:
                        segments[rotation(rot + 0)] = terrain
                        segments[rotation(rot + 1)] = terrain
                        segments[rotation(rot + 2)] = terrain
                        segments[rotation(rot + 3)] = terrain
                        segments[rotation(rot + 4)] = terrain
                        segments[rotation(rot + 5)] = terrain
                    case Form.unknown_102:
                        pass
                    case Form.unknown_105:
                        pass
                    case Form.water_size4:
                        segments[rotation(rot + 0)] = terrain
                        segments[rotation(rot + 1)] = terrain
                        segments[rotation(rot + 2)] = terrain
                        segments[rotation(rot + 3)] = terrain
                    case Form.unknown_111:
                        pass

                terrains.add(segment.terrain)
                forms.add(segment.form)

            special.add(tile.special_tile.id)

            num_tile_4bytes = 2 + 6
            base_index = num_tile_4bytes * index
            tiles_data[base_index] = tile.s
            tiles_data[base_index + 1] = tile.t
            for i, s in enumerate(segments):
                tiles_data[base_index + 2 + i] = s

        self.tiles_buffer.write(tiles_data)

        self.log(f"Terrains: {terrains}")
        self.log(f"Forms {forms}")
        self.log(f"Special {special}")

        chunk_headers_data = numpy.array(chunks.headers, dtype=numpy.int32)
        self.chunk_headers_buffer.write(chunk_headers_data)

        chunk_indices_data = numpy.array(chunks.indices, dtype=numpy.int32)
        self.chunk_indices_buffer.write(chunk_indices_data)

    def rebind_buffers(self):
        self.log("Rebinding")
        program = self.shader
        if not program.program:
            self.log("Invalid shader not rebinding buffers")
            return

        self.globals_buffer.rebind(program)
        self.view_buffer.rebind(program)
        self.chunk_meta_buffer.rebind(program)
        self.tiles_buffer.rebind(program)
        self.chunk_headers_buffer.rebind(program)
        self.chunk_indices_buffer.rebind(program)

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

    def set_data(self, state: State, chunks: Chunks):
        self.state = state
        self.chunks = chunks

        self.create_buffers()
        self.rebind_buffers()
        self.initialize_buffers()
        self.reset()

    def write_globals_buffer(self):
        globals_data = numpy.array([self.current_time()], dtype=numpy.float32)
        self.globals_buffer.write(globals_data)

    def write_view_buffer(self):
        fovy = math.pi / 2
        view_data = numpy.array([fovy, 0, *self.origin, 0, *self.right, 0, *self.ahead, 0, *self.up, 0], dtype=numpy.float32)
        self.view_buffer.write(view_data, offset=2 * 4)

    def reset(self):
        self.origin = numpy.array([100.0, 0.0, 100.0])
        # self.set_yaw_pitch(0, -math.pi / 2)
        self.set_yaw_pitch(7 * math.pi / 8, 0)

        self.lock_mouse(False)

    def update(self):
        pressed = pygame.key.get_pressed()

        movement = numpy.array([0.0, 0.0, 0.0])
        movement += self.ahead * (int(pressed[pygame.K_w]) - int(pressed[pygame.K_s]))
        movement += self.right * (int(pressed[pygame.K_d]) - int(pressed[pygame.K_a]))
        movement_sqr = numpy.dot(movement, movement)
        if movement_sqr > 0.5:
            movement = movement / math.sqrt(movement_sqr) * 0.1
            factor = 2 if pressed[pygame.K_LSHIFT] else 1

            self.origin += factor * movement

    def render(self):
        if self.shader.was_source_modified_since():
            self.log("Recompiling shaders")
            if self.shader.reinstall_if_valid():
                self.rebind_buffers()

        self.write_globals_buffer()
        self.write_view_buffer()
        GL.glDrawArrays(GL.GL_TRIANGLE_STRIP, 0, 4);


if __name__ == "__main__":
    print("Loading GUI")
    game = Dorfromantik()

    print("Parsing savegame")
    data = parse_savegame(sys.argv[1:])

    print("Interpreting data")
    state = State.from_dump(data)

    print("Building chunks")
    tile_items = [(index, tile.s, tile.t) for index, tile in enumerate(state.tiles)]
    chunks = Chunks(tile_items, chunk_size=4)

    print("Running game")
    game.set_data(state, chunks)
    game.reset()
    game.run()
