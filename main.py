# https://stackoverflow.com/questions/3052202/how-to-analyse-contents-of-binary-serialization-stream/30176566#30176566

import sys
from dorfromantischer.savegame import parse_savegame
from dorfromantischer.state import State, Form, Terrain
from dorfromantischer.quadtree import Quadtree, Pos
from dorfromantischer.game3d import Game3D
from dorfromantischer.gl.program import Program
from dorfromantischer.gl.buffer import Buffer, UniformBuffer, ShaderStorageBuffer
import pygame
import math
from OpenGL import GL
import numpy
from scipy.spatial.transform import Rotation


def normalize(vector: numpy.ndarray):
    return vector / numpy.linalg.norm(vector)


# Byte sizes.
int_ = 4
float_ = 4
vec3_ = (3 + 1) * float_


form_mapping = {
    Form.size1: [0],
    Form.size2: [0, 1],
    Form.bridge: [0, 2],
    Form.straight: [0, 3],
    Form.size3: [0, 1, 2],
    Form.junction_left: [0, 1, 3],
    Form.junction_right: [0, 1, 4],
    Form.three_way: [0, 2, 4],
    Form.size4: [0, 1, 2, 3],
    Form.fan_out: [0, 1, 2, 4],
    Form.x: [0, 1, 3, 4],
    Form.size5: [0, 1, 2, 3, 4],
    Form.size6: [0, 1, 2, 3, 4, 5],
    Form.unknown_102: [],
    Form.unknown_105: [],
    Form.water_size4: [0, 1, 2, 3],
    Form.unknown_111: [],
}

class Dorfromantik(Game3D):
    state: State
    quadtree: Quadtree

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
    tiles_buffer: Buffer
    quadtree_buffer: Buffer

    def __init__(self):
        super().__init__()

        self.shader = Program("shaders/shader.vert", "shaders/shader.frag")

    def create_buffers(self):
        self.log("Create buffers")
        state = self.state
        tiles = state.tiles
        quadtree = self.quadtree

        # Globals buffer
        globals_bytes = 1 * float_
        self.globals_buffer = UniformBuffer(globals_bytes, "globals_buffer")

        # View buffer
        view_bytes = 2 * int_ + 2 * float_ + 4 * vec3_
        self.view_buffer = UniformBuffer(view_bytes, "view_buffer")

        # Tiles buffer
        tile_bytes = 2 * int_ + 6 * int_
        tiles_bytes = len(tiles) * tile_bytes
        self.tiles_buffer = ShaderStorageBuffer(tiles_bytes, "tiles_buffer")

        # Quadtree buffer
        quadtree_node_bytes = 4 * int_
        quadtree_bytes = (3 + 1) * int_ + len(quadtree.nodes) * quadtree_node_bytes
        self.quadtree_buffer = ShaderStorageBuffer(quadtree_bytes, "quadtree_buffer")

    def initialize_buffers(self):
        self.log("Initialize buffers")
        state = self.state
        tiles = state.tiles
        quadtree = self.quadtree

        # View data
        view_data = numpy.array([self.WIDTH, self.HEIGHT], dtype=numpy.int32)
        self.view_buffer.write(view_data)
        view_data = numpy.array([math.pi / 2, 0], dtype=numpy.float32)
        self.view_buffer.write(view_data, offset=2 * int_)

        # Tiles data
        terrains = set()
        forms = set()
        special = set()

        def rotation(rot):
            return rot % 6

        tiles_data = numpy.zeros(shape=(len(tiles), 2 + 6), dtype=numpy.int32)
        for index, tile in enumerate(tiles):
            segments = [Terrain.empty.value] * 6

            for segment in tile.segments:
                rot = rotation(tile.rotation + segment.rotation)
                terrain = segment.terrain.value

                for index in form_mapping[segment.form]:
                    segments[rotation(rot + index)] = terrain

                terrains.add(segment.terrain)
                forms.add(segment.form)

            special.add(tile.special_tile.id)

            tiles_data[index] = [tile.s, tile.t, *segments]

        self.log(f"Terrains: {terrains}")
        self.log(f"Forms {forms}")
        self.log(f"Special {special}")

        self.tiles_buffer.write(tiles_data)

        # Quadtree data
        quadtree_data = numpy.array([quadtree.root_s, quadtree.root_t, quadtree.root_width], dtype=numpy.int32)
        self.quadtree_buffer.write(quadtree_data)
        quadtree_data = numpy.array(quadtree.nodes, dtype=numpy.int32)
        self.quadtree_buffer.write(quadtree_data, offset =(3 + 1) * int_)

    def rebind_buffers(self):
        self.log("Rebinding")
        program = self.shader
        if not program.program:
            self.log("Invalid shader not rebinding buffers")
            return

        self.globals_buffer.rebind(program)
        self.view_buffer.rebind(program)
        self.tiles_buffer.rebind(program)
        self.quadtree_buffer.rebind(program)

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

    def set_data(self, state: State, quadtree: Quadtree):
        self.state = state
        self.quadtree = quadtree

        self.create_buffers()
        self.rebind_buffers()
        self.initialize_buffers()
        self.reset()

    def write_globals_buffer(self):
        globals_data = numpy.array([self.current_time()], dtype=numpy.float32)
        self.globals_buffer.write(globals_data)

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

    print("Building quadtree")
    tile_items = [(Pos(tile.s, tile.t), index)  for index, tile in enumerate(state.tiles)]
    quadtree = Quadtree(tile_items)

    print("Running game")
    game.set_data(state, quadtree)
    game.reset()
    game.run()
