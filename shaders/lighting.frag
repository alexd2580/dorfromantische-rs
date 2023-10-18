#version 460

in vec2 uv;
out vec4 frag_color;

layout(binding=0) uniform sampler2D data;

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

#define RUNOFF 0
#define TOO_MANY_STEPS 1
#define UNDEFINED 2
#define SKYBOX 3

#define TRUNK 10
#define TREETOP 11
#define HOUSE_BASE 12
#define HOUSE_ROOF 13
#define FLOOR 14

vec3 material_color_of(int material) {
    switch(material) {
    case RUNOFF:
        return vec3(1, 0, 1);
    case TOO_MANY_STEPS:
        return vec3(0);
    case UNDEFINED:
        return vec3(1, 0, 1);
    case SKYBOX:
        return vec3(0.7, 0.7, 1.0);
    case TRUNK:
        return vec3(0.42, 0.35, 0.25);
    case TREETOP:
        return vec3(0.30, 0.34, 0.17);
    case HOUSE_BASE:
        return vec3(0.89, 0.75, 0.47);
    case HOUSE_ROOF:
        return vec3(0.96, 0.26, 0.26);
    case FLOOR:
        return vec3(0.2);
    }
}

void main(){
    float aspect_ratio = float(size.s) / size.t;
    float fov_factor = tan(0.5 * fovy);

    vec2 uv_a = uv;
    vec2 uv_b = uv_a + vec2(1.0 / size.x, 0);
    vec2 uv_c = uv_a + vec2(0, 1.0 / size.y);

    vec3 dir_a = normalize(camera_ahead + fov_factor * (uv_a.x * aspect_ratio * camera_right + uv_a.y * camera_up));
    vec3 dir_b = normalize(camera_ahead + fov_factor * (uv_b.x * aspect_ratio * camera_right + uv_b.y * camera_up));
    vec3 dir_c = normalize(camera_ahead + fov_factor * (uv_c.x * aspect_ratio * camera_right + uv_c.y * camera_up));

    vec4 pixel_data = texture(data, 0.5 * uv_a + 0.5);
    float distance_a = pixel_data.x * 500.0;
    vec3 material_color = material_color_of(int(pixel_data.z * 100.0));

    if (distance_a > 100000) {
        frag_color = vec4(material_color, 1);
        return;
    }
    float distance_b = 500.0 * texture(data, 0.5 * uv_b + 0.5).r;
    float distance_c = 500.0 * texture(data, 0.5 * uv_c + 0.5).r;

    vec3 a = dir_a * distance_a;
    vec3 ab = a - distance_b * dir_b;
    vec3 ac = a - distance_c * dir_c;
    vec3 normal = normalize(cross(ab, ac));

    // TODO make global.
    // 2 revolutions per minute.
    float light_time = 0.026 * time / (2*pi);
    vec3 light_dir = normalize(vec3(cos(light_time), 1, sin(light_time)));

    float ambient = 0.5;
    float diffuse = max(2.0 * dot(normal, light_dir), 0.0);
    vec3 half_light_view = normalize(light_dir - dir_a);
    float specular = max(1.0 * pow(dot(normal, half_light_view), 200.0), 0.0);
    float lightness = ambient + diffuse + specular;

    float shadow = pixel_data.y * diffuse;
    frag_color = vec4(material_color * (ambient + shadow), 1);
}
