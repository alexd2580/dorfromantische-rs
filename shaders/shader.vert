#version 460 core

vec2 quadVertices[4] = { vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(-1.0, 1.0), vec2(1.0, 1.0) };

out vec2 uv;

void main() {
    uv = quadVertices[gl_VertexID];
    gl_Position = vec4(uv, 0.0, 1.0);
}
