#version 450

in vec2 uv;
out vec4 frag_color;

uniform sampler2D color;
uniform sampler2D depth;

layout (std140) uniform globals_buffer {
    float time;
    ivec4 quadrant_sizes;
};

layout (std140) uniform view_buffer {
    ivec2 size;

    float fovy;
    float _pad;

    vec3 camera_origin;
    vec3 camera_right;
    vec3 camera_ahead;
    vec3 camera_up;
};

struct Ray {
    vec3 origin;
    vec3 dir;
    vec3 inv_dir;
};

const float inf = 1.0 / 0.0;
const float pi = 3.141592653589793;
const float epsilon = 0.00001;

void main(){
    float aspect_ratio = float(size.s) / size.t;

    vec3 ray_origin = camera_origin;
    float fov_factor = tan(0.5 * fovy);

    vec3 ray_dir = normalize(camera_ahead + fov_factor * (uv.x * aspect_ratio * camera_right + uv.y * camera_up));
    Ray ray = Ray(ray_origin, ray_dir, 1 / ray_dir);
    frag_color = vec4(vec3(texture(depth, uv)), 1);

    // vec3 ambient = 0.1 * intersection.color;
    // vec3 light = vec3(0.577);
    // vec3 diffuse = vec3(max(2.0 * dot(intersection.normal, light), 0.0));
    // vec3 half_light_view = normalize(light - ray.dir);
    // vec3 specular = vec3(max(1.0 * pow(dot(intersection.normal, half_light_view), 200.0), 0.0));
    // vec3 lightness = ambient + diffuse + specular;

    // frag_color = vec4(intersection.color * lightness, 1);
}
