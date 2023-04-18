import os
import pathlib
from OpenGL import GL
from typing import Optional


def compile_shader(shader_type, source_path: pathlib.Path) -> Optional[GL.GLuint]:
    source = source_path.read_text()

    if (shader := GL.glCreateShader(shader_type)) == 0:
        return None

    GL.glShaderSource(shader, source)
    GL.glCompileShader(shader)

    if GL.glGetShaderiv(shader, GL.GL_COMPILE_STATUS, None) == GL.GL_FALSE:
        info_log = GL.glGetShaderInfoLog(shader)
        print(info_log.decode())
        GL.glDeleteShader(shader)
        return None

    return shader


def link_program(shaders: list[GL.GLuint]) -> Optional[GL.GLuint]:
    if (program := GL.glCreateProgram()) == 0:
        return None

    for shader in shaders:
        GL.glAttachShader(program, shader)
    GL.glLinkProgram(program)

    if GL.glGetProgramiv(program, GL.GL_LINK_STATUS, None) == GL.GL_FALSE:
        info_log = GL.glGetProgramInfoLog(program)
        print(info_log.decode())
        GL.glDeleteProgram(program)
        return None

    return program


def get_mtime(path: pathlib.Path) -> float:
    return path.stat().st_mtime


class Program:
    vertex_shader_path: pathlib.Path
    vertex_shader_mtime: float
    vertex_shader: Optional[GL.GLuint]

    fragment_shader_path: pathlib.Path
    fragment_shader_mtime: float
    fragment_shader: Optional[GL.GLuint]

    program: Optional[GL.GLuint]

    def __init__(self, vertex_shader_path: str, fragment_shader_path: str):
        self.vertex_shader_path = pathlib.Path(vertex_shader_path)
        self.vertex_shader_mtime = 0
        self.vertex_shader = None

        self.fragment_shader_path = pathlib.Path(fragment_shader_path)
        self.fragment_shader_mtime = 0
        self.fragment_shader = None

        self.program = None
        self.reinstall_if_valid()

    def _compile_vertex_shader(self) -> Optional[GL.GLuint]:
        self.vertex_shader_mtime = get_mtime(self.vertex_shader_path)
        return compile_shader(GL.GL_VERTEX_SHADER, self.vertex_shader_path)

    def _compile_fragment_shader(self) -> Optional[GL.GLuint]:
        self.fragment_shader_mtime = get_mtime(self.fragment_shader_path)
        return compile_shader(GL.GL_FRAGMENT_SHADER, self.fragment_shader_path)

    def reinstall_if_valid(self) -> bool:
        if vertex_shader := self._compile_vertex_shader():
            if fragment_shader := self._compile_fragment_shader():
                print(vertex_shader, fragment_shader)
                if program := link_program([vertex_shader, fragment_shader]):
                    self.destroy()
                    self.vertex_shader = vertex_shader
                    self.fragment_shader = fragment_shader
                    self.program = program
                    GL.glUseProgram(self.program)
                    return True

                GL.glDeleteShader(fragment_shader)
            GL.glDeleteShader(vertex_shader)
        return False


    def destroy(self):
        if self.program is not None:
            GL.glUseProgram(0)
            GL.glDeleteProgram(self.program)
            self.program = None
        if self.fragment_shader is not None:
            GL.glDeleteShader(self.fragment_shader)
            self.fragment_shader = None
        if self.vertex_shader is not None:
            GL.glDeleteShader(self.vertex_shader)
            self.vertex_shader = None

    def was_source_modified_since(self) -> bool:
        vertex_shader_modified = get_mtime(self.vertex_shader_path) > self.vertex_shader_mtime
        fragment_shader_modified = get_mtime(self.fragment_shader_path) > self.fragment_shader_mtime
        return vertex_shader_modified or fragment_shader_modified
