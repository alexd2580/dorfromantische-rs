#version 460

in vec2 uv;
out vec4 frag_color;

layout(binding=0) uniform sampler2D color;
layout(binding=1) uniform sampler2D depth;

layout(std140) uniform globals_buffer {
    float time;
    ivec4 quadrant_sizes;
};

layout(std140) uniform view_buffer {
    ivec2 size;

    float fovy;
    float _pad;

    vec3 camera_origin;
    vec3 camera_right;
    vec3 camera_ahead;
    vec3 camera_up;
};

const float inf = 1.0 / 0.0;
const float pi = 3.141592653589793;
const float epsilon = 0.00001;

void main(){
    float aspect_ratio = float(size.s) / size.t;
    float fov_factor = tan(0.5 * fovy);

    vec2 uv_a = uv;
    vec2 uv_b = uv_a + vec2(1.0 / size.x, 0);
    vec2 uv_c = uv_a + vec2(0, 1.0 / size.y);

    vec3 dir_a = normalize(camera_ahead + fov_factor * (uv_a.x * aspect_ratio * camera_right + uv_a.y * camera_up));
    vec3 dir_b = normalize(camera_ahead + fov_factor * (uv_b.x * aspect_ratio * camera_right + uv_b.y * camera_up));
    vec3 dir_c = normalize(camera_ahead + fov_factor * (uv_c.x * aspect_ratio * camera_right + uv_c.y * camera_up));

    float depth_a = 300.0 * texture(depth, 0.5 * uv_a + 0.5).r;
    if (depth_a > 100000) {
        vec3 color = texture(color, 0.5 * uv_a + 0.5).rgb;
        frag_color = vec4(color, 1);
        return;
    }
    float depth_b = 300.0 * texture(depth, 0.5 * uv_b + 0.5).r;
    float depth_c = 300.0 * texture(depth, 0.5 * uv_c + 0.5).r;

    vec3 a = dir_a * depth_a;
    vec3 ab = a - depth_b * dir_b;
    vec3 ac = a - depth_c * dir_c;
    vec3 normal = normalize(cross(ab, ac));

    vec3 light = vec3(0.577);

    float ambient = 0.5;
    float diffuse = max(2.0 * dot(normal, light), 0.0);
    vec3 half_light_view = normalize(light - dir_a);
    float specular = max(1.0 * pow(dot(normal, half_light_view), 200.0), 0.0);
    float lightness = ambient + diffuse + specular;

    vec3 color = texture(color, 0.5 * uv_a + 0.5).rgb;
    frag_color = vec4(color, 1);
}
