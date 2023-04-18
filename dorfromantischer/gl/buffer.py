from OpenGL import GL
from dorfromantischer.gl.program import Program
import numpy as np
from contextlib import contextmanager


@contextmanager
def bind_buffer(buffer_type: GL.GLuint, buffer: GL.GLuint):
    GL.glBindBuffer(buffer_type, buffer)
    yield
    GL.glBindBuffer(buffer_type, 0)


class Buffer:
    # Class variables.
    resource_type: GL.GLuint
    buffer_type: GL.GLuint
    next_free_binding: int

    byte_size: int
    buffer_name: str

    block_binding: int
    buffer: GL.GLuint

    def __init__(self, buffer_byte_size: int, buffer_name: str):
        self.buffer_byte_size = buffer_byte_size
        self.buffer_name = buffer_name

        self.block_binding = self.__class__.next_free_binding
        self.__class__.next_free_binding += 1

        buffer_type = self.__class__.buffer_type

        self.buffer = GL.glGenBuffers(1)
        with bind_buffer(buffer_type, self.buffer):
            GL.glBufferData(buffer_type, self.buffer_byte_size, None, GL.GL_STATIC_DRAW);
        GL.glBindBufferBase(buffer_type, self.block_binding, self.buffer);

    def _block_binding(self, program: Program, block_index: GL.GLuint):
        del program, block_index
        raise NotImplementedError("Something went wrong, ask your local nerd, lol.")

    def rebind(self, program: Program):
        block_index = GL.glGetProgramResourceIndex(program.program, self.__class__.resource_type, self.buffer_name)
        self._block_binding(program, block_index)

    def write(self, data: np.ndarray, offset: int = 0):
        buffer_type = self.__class__.buffer_type
        with bind_buffer(buffer_type, self.buffer):
            GL.glBufferSubData(buffer_type, offset, data.nbytes, data);


class UniformBuffer(Buffer):
    resource_type = GL.GL_UNIFORM_BLOCK
    buffer_type = GL.GL_UNIFORM_BUFFER

    next_free_binding = 0

    def _block_binding(self, program: Program, block_index: GL.GLuint):
        GL.glUniformBlockBinding(program.program, block_index, self.block_binding)


class ShaderStorageBuffer(Buffer):
    resource_type = GL.GL_SHADER_STORAGE_BLOCK
    buffer_type = GL.GL_SHADER_STORAGE_BUFFER

    next_free_binding = 0

    def _block_binding(self, program: Program, block_index: GL.GLuint):
        GL.glShaderStorageBlockBinding(program.program, block_index, self.block_binding)
