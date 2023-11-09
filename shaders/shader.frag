#version 460

in vec2 uv;

layout(location=0) out vec4 frag_data;

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

struct Tile {
    bool present;
    int special_id;

    /**
     * Interleaved
     * - form (enum value); 0 means none
     * - terrain (enum value)
     * - rotation (int)
     */
    int segments[18];
    int _pad4[4];
};

#define EMPTY_SEGMENT -1
#define HOUSE_SEGMENT 0
#define FOREST_SEGMENT 1
#define WHEAT_SEGMENT 2
#define RAIL_SEGMENT 3
#define WATER_SEGMENT 4

#define FORM_SIZE1 = 1
#define FORM_SIZE2 = 2
#define FORM_BRIDGE = 3 // 1-skip1-1
#define FORM_STRAIGHT = 4 // 1-skip2-1
#define FORM_SIZE3 = 5
#define FORM_JUNCTION_LEFT = 6 // 2-skip1-1
#define FORM_JUNCTION_RIGHt = 7 // 2-skip2-1
#define FORM_THREE_WAY = 8 // 1-skip1-1-skip1-1
#define FORM_SIZE4 = 9
#define FORM_FAN_OUT = 10 // 3-skip1-1
#define FORM_X = 11 // 2-skip1-2
#define FORM_SIZE5 = 12
#define FORM_SIZE6 = 13
#define FORM_
#define FORM_UNKNOWN_102 = 14
#define FORM_UNKNOWN_105 = 15
#define FORM_WATER_SIZE4 = 16
#define FORM_UNKNOWN_111 = 17

layout(std430) buffer quadrant0_buffer {
    Tile quadrant0[];
};
layout(std430) buffer quadrant1_buffer {
    Tile quadrant1[];
};
layout(std430) buffer quadrant2_buffer {
    Tile quadrant2[];
};
layout(std430) buffer quadrant3_buffer {
    Tile quadrant3[];
};

struct Ray {
    vec3 origin;
    vec3 dir;
    vec3 inv_dir;
};

const float inf = 1.0 / 0.0;
const float pi = 3.141592653589793;
const float epsilon = 0.00001;
const float omega = 10000000;

// PI / 6
const float deg_30 = pi * 0.166666666;
const float cos_30 = cos(deg_30);
const float sin_30 = 0.5;

const vec3 tile_d = vec3(2 * cos_30, 0, 1.5);
const vec2 no_intersection = vec2(inf, 0);

const int ray_march_steps = 40;
const float ray_march_eps = 0.00001;

#define RUNOFF 0
#define TOO_MANY_STEPS 1
#define UNDEFINED 2
#define SKYBOX 3

#define TRUNK 10
#define TREETOP 11
#define HOUSE_BASE 12
#define HOUSE_ROOF 13
#define FLOOR 14
#define WHEAT 15
#define WATER 16

float rand(vec2 seed) {
    // return seed.x;
    return fract(sin(dot(seed, vec2(12.9898, 78.233))) * 43758.5453);
}

vec3 syt_to_xyz(int s, float y, int t) {
    return vec3(t * tile_d.x - 0.5 * (s % 2) * tile_d.x, y, s * tile_d.z);
}

float distance_to_y(Ray ray, float target_y) {
    return (target_y - ray.origin.y) * ray.inv_dir.y;
}

// Suddenly ray marching!
float maxcomp(vec3 vec) {
    return max(vec.x, max(vec.y, vec.z));
}

vec3 at(vec3 pos, vec3 o) {
    return pos - o;
}

vec3 rotate_x(vec3 pos, float rad) {
    float s = sin(rad), c = cos(rad);
    return vec3(
        pos.x,
        s * pos.z + c * pos.y,
        c * pos.z - s * pos.y
    );
}

vec3 rotate_y(vec3 pos, float rad) {
    float s = sin(rad), c = cos(rad);
    return vec3(
        c * pos.x - s * pos.z,
        pos.y,
        s * pos.x + c * pos.z
    );
}

vec3 rotate_z(vec3 pos, float rad) {
    float s = sin(rad), c = cos(rad);
    return vec3(
        s * pos.y + c * pos.x,
        c * pos.y - s * pos.x,
        pos.z
    );
}

// Repetition.

vec3 repeated_x(vec3 pos, float step) {
    return vec3(pos.x - step * round(pos.x / step), pos.y, pos.z);
}

vec3 repeated_xz(vec3 pos, float step) {
    vec2 xz = pos.xz - step * round(pos.xz / step);
    return vec3(xz.x, pos.y, xz.y);
}

#define repeated_xz_variate(pos_var, index_var, step_var, scene_sdf_func, result_var) \
    vec3 index_var = round(pos_var / step_var); \
    vec2 result_var = vec2(inf, 1); \
    for(int i = -1; i < 2; i++) { for(int j = -1; j < 2; j++) { \
        vec3 local_index = index_var + vec3(i, 0, j); \
        vec2 temp = scene_sdf_func(pos_var - step_var * local_index, local_index); \
        result_var = temp.x < result_var.x ? temp : result_var; \
    }}

// Combinators.

vec2 sdf_intersect(vec2 a, vec2 b) { return vec2(max(a.x, b.x), a.y); }
vec2 sdf_union(vec2 a, vec2 b) { return a.x < b.x ? a : b; }
vec2 sdf_subtract(vec2 a, vec2 b) { return vec2(max(-a.x, b.x), a.y); }

// Primitives.

vec2 sdf_box(vec3 pos, vec3 side_len) {
    vec3 to_corner = abs(pos) - side_len / 2;
    return vec2(length(max(to_corner, 0)) + min(maxcomp(to_corner), 0), 1);
}

vec2 sdf_plane(vec3 pos, vec3 normal, float offset) { return vec2(dot(pos, normal) - offset, 1); }
vec2 sdf_sphere(vec3 pos, float r) { return vec2(length(pos) - r, 1); }
vec2 sdf_cylinder_x(vec3 pos, float r) { return vec2(length(pos.yz) - r, 1); }
vec2 sdf_cylinder_y(vec3 pos, float r) { return vec2(length(pos.xz) - r, 1); }
vec2 sdf_cylinder_z(vec3 pos, float r) { return vec2(length(pos.xy) - r, 1); }

vec2 sdf_cone_y(vec3 pos, vec2 c) {
    // This is code from iquilezles.org
    // c is the sin/cos of the angle
    vec2 q = vec2(length(pos.xz), -pos.y);
    float d = length(q - c * max(dot(q, c), 0.0));
    return vec2(d * ((q.x * c.y - q.y * c.x < 0.0) ? -1.0 : 1.0), 1);
}

// Composed structures.

vec2 sdf_floor(vec3 pos) {
    return sdf_plane(pos, vec3(0, 1, 0), 0) * vec2(1, FLOOR);
}

vec2 sdf_tile_bounds(vec3 pos) {
    // We unmirror the hex, so that only the top left quadrant has to be checked.
    pos = vec3(abs(pos.x), pos.y, abs(pos.z));

    vec2 bottom = sdf_plane(pos, vec3(0, -1, 0), 0.1);
    vec2 front_tr = sdf_intersect(
        sdf_plane(pos, vec3(sin_30, 0, cos_30), 0.95 * cos_30),
        sdf_plane(pos, vec3(1, 0, 0), 0.95 * cos_30)
    );
    return sdf_intersect(bottom, front_tr);
}

vec2 sdf_roof(vec3 pos) {
    vec2 fb = sdf_intersect(
        sdf_plane(pos, vec3(sin_30, cos_30, 0), sin_30 + 0.1),
        sdf_plane(pos, vec3(-sin_30, cos_30, 0), sin_30 + 0.1)
    );
    vec2 lr = sdf_intersect(
        sdf_plane(pos, vec3(0, cos_30, sin_30), sin_30 + 0.1),
        sdf_plane(pos, vec3(0, cos_30, -sin_30), sin_30 + 0.1)
    );
    vec2 bottom = vec2(-pos.y, 1);

    return sdf_intersect(sdf_intersect(fb, lr), bottom) * vec2(1, HOUSE_ROOF);
}

vec2 sdf_house(vec3 pos, vec3 index) {
    float scale = 0.03 + 0.04 * rand(index.xz);
    pos = at(pos, 0.12 * vec3(rand(index.xz), 0, rand(index.zx)) - vec3(0.06, 0, 0.06)) / scale;

    float r = rand(index.xz + 1);
    pos = rotate_y(pos, r * 2 * pi);

    vec2 base = sdf_box(at(pos, vec3(0, 0.75, 0)), vec3(2, 1.5, 2));
    base.y = HOUSE_BASE;
    vec2 roof = sdf_roof(at(pos, vec3(0, 1.5, 0)));
    roof.y = HOUSE_ROOF;
    return sdf_union(base, roof) * vec2(scale, 1);
}

vec2 sdf_town(vec3 pos) {
    // vec3 house_offset = vec3(0.5 * cos_30, 0, 0);
    // vec3 repeated_pos = repeated_xz(at(pos, house_offset) / scale, 5);

    // Offset to triangle center.
    repeated_xz_variate(pos, index, vec3(0.2, 0, 0.2), sdf_house, house)

    return house;
}

vec2 sdf_tree(vec3 pos, vec3 index) {
    float scale = 0.5 + 0.5 * rand(index.xz);

    pos /= scale;
    pos += 0.9 * vec3(rand(index.xz) - 0.5, 0, rand(index.xz + 1) - 0.5);

    // 1 cycle per 5 seconds.
    float tree_time = 0.2 * time / (2*pi) * (rand(index.xz) / 2 + 0.75);
    float sway_time = 5 * tree_time * (rand(index.xz) / 2 + 0.75);
    // pos *= 1 + 0.2 * vec3(sin(tree_time), 0, cos(sway_time));
    pos = rotate_x(rotate_z(pos, 0.2 * cos(tree_time)), 0.2 * cos(sway_time));

    vec2 trunk_cyl = sdf_cylinder_y(pos, 0.15);
    vec2 trunk_min = sdf_plane(pos, vec3(0, -1, 0), 0);
    vec2 trunk_max = sdf_plane(pos, vec3(0, 1, 0), 0.5);
    vec2 trunk = sdf_intersect(trunk_min, sdf_intersect(trunk_cyl, trunk_max));
    trunk.y = TRUNK;

    vec2 treetop_cone = sdf_cone_y(at(pos, vec3(0, 2, 0)), vec2(sin(0.6 * deg_30), cos(0.6 * deg_30)));
    vec2 treetop_min = sdf_plane(pos, vec3(0, -1, 0), -0.5);
    vec2 treetop = sdf_intersect(treetop_cone, treetop_min);
    treetop.y = TREETOP;

    return sdf_union(trunk, treetop) * vec2(scale, 1);
}

vec2 sdf_forest(vec3 pos) {
    float scale = 0.2;
    pos /= scale;
    repeated_xz_variate(pos, index, vec3(1, 0, 1), sdf_tree, tree)
    return tree * vec2(scale, 1);
}

vec2 sdf_wheat(vec3 pos, vec3 index) {
    pos += 0.02 * vec3(rand(index.xz) - 0.5, 0, rand(index.xz + 1) - 0.5);

    // 1 cycle per second.
    float tree_time = time / (2*pi) * (rand(index.xz) / 2 + 0.75);
    float sway_time = 5 * tree_time * (rand(index.xz) / 2 + 0.75);
    pos = rotate_x(rotate_z(pos, 0.2 * cos(tree_time)), 0.2 * cos(sway_time));

    return sdf_cylinder_y(pos, 0.003);
}

vec2 sdf_wheat_field(vec3 pos) {
    float scale = 3;
    repeated_xz_variate(pos / scale, index, vec3(0.01, 0, 0.01), sdf_wheat, wheat)
    return sdf_intersect(wheat * vec2(1, WHEAT), sdf_plane(pos, vec3(0, 1, 0), 0.07)) * (scale, 1);
}

// SdfHit sdf_rail_tile(vec3 origin) {
//     // Do everything in scaled coordinates.
//     float scale = 0.1;
//     vec3 scaled_pos = origin / scale;
//
//     // Create a pair of rails.
//     float rail_radius = 0.05;
//     vec3 rail_offset = vec3(0, 0, 0.5);
//     SdfHit rail_l = sdf_cylinder_x(at(scaled_pos, +rail_offset), rail_radius);
//     SdfHit rail_r = sdf_cylinder_x(at(scaled_pos, -rail_offset), rail_radius);
//
//     SdfHit rails = sdf_union(rail_l, rail_r);
//     rails.color = vec3(0.71, 0.25, 0.05);
//
//     // Repeat sleepers.
//     vec3 pos_repeated_x = repeated_x(origin / scale, 0.5);
//     SdfHit sleeper = sdf_cylinder_z(pos_repeated_x, 0.7 * rail_radius);
//     sleeper.color = vec3(0.4);
//
//     // Join stuff together.
//     SdfHit rails_and_sleeper = sdf_union(rails, sleeper);
//
//     // Size is not radius, it's the extent in that direction.
//     // X is unscaled early, rest is in scaled coordinates.
//     vec3 bounds_pos = vec3(0.5 * cos_30 / scale, 0, 0);
//     vec3 bounds_size = vec3(cos_30 / scale, 1, 1.4);
//     SdfHit rail_bounds = sdf_box(at(scaled_pos, bounds_pos), bounds_size);
//     SdfHit bounded_rail = sdf_intersect(rails_and_sleeper, rail_bounds);
//
//     // Undo scaling.
//     bounded_rail.distance *= scale;
//     return bounded_rail;
// }

vec2 sdf_water(vec3 pos) {
    return sdf_plane(pos, vec3(0, 1, 0), 0.01) * vec2(1, WATER);
}

// SdfHit sdf_bahnhof_tile(vec3 pos) {
//     SdfHit building = sdf_sphere(pos, 1);
//     building.color = vec3(0.2, 0.3, 0.5);
//
//     SdfHit horizontal = sdf_intersect(
//         sdf_plane(pos, vec3(-sin_30, 0, cos_30), 0),
//         sdf_plane(pos, vec3(-sin_30, 0, -cos_30), 0)
//     );
//     SdfHit front = sdf_plane(pos, vec3(1, 0, 0), 0.95 * cos_30);
//     SdfHit bounds = sdf_intersect(horizontal, front);
//
//     return sdf_intersect(bounds, building);
// }

Tile get_tile(ivec2 st) {
    // TODO check indices!
    int quadrant = st.s >= 0 ? (st.t >= 0 ? 0 : 3) : (st.t >= 0 ? 1 : 2);
    int _s = st.s >= 0 ? st.s : (-1 - st.s);
    int _t = st.t >= 0 ? st.t : (-1 - st.t);

    int index = int((_s + _t + 1) * (_s + _t) / 2.0) + _t;

    if (quadrant == 0 && index < quadrant_sizes.x) {
        return quadrant0[index];
    } else if (quadrant == 1 && index < quadrant_sizes.y) {
        return quadrant1[index];
    } else if (quadrant == 2 && index < quadrant_sizes.z) {
        return quadrant2[index];
    } else if (quadrant == 3 && index < quadrant_sizes.w) {
        return quadrant3[index];
    }

    return Tile(false, 0, int[18](0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), int[4](0, 0, 0, 0));
}

vec2 sdf_segment_border(vec3 pos, int form, int rotation) {
    switch(form) {
        case FORM_SIZE1:
            return sdf_sphere(at(pos, ), );
        case FORM_SIZE2 = 2
        case FORM_BRIDGE = 3 // 1-skip1-1
        case FORM_STRAIGHT = 4 // 1-skip2-1
        case FORM_SIZE3 = 5
        case FORM_JUNCTION_LEFT = 6 // 2-skip1-1
        case FORM_JUNCTION_RIGHt = 7 // 2-skip2-1
        case FORM_THREE_WAY = 8 // 1-skip1-1-skip1-1
        case FORM_SIZE4 = 9
        case FORM_FAN_OUT = 10 // 3-skip1-1
        case FORM_X = 11 // 2-skip1-2
        case FORM_SIZE5 = 12
        case FORM_SIZE6 = 13
        case FORM_
        case FORM_UNKNOWN_102 = 14
        case FORM_UNKNOWN_105 = 15
        case FORM_WATER_SIZE4 = 16
        case FORM_UNKNOWN_111 = 17
    }
    return vec2(0, 0);
}

vec2 sdf_segment(vec3 global_pos, vec3 pos, int form, int terrain, int rotation) {
    switch(terrain) {
    case HOUSE_SEGMENT:
        return sdf_intersect(sdf_town(global_pos), sdf_segment_border(pos, form, rotation));
    case FOREST_SEGMENT:
        return sdf_intersect(sdf_forest(global_pos), sdf_segment_border(pos, form, rotation));
    case WHEAT_SEGMENT:
        return sdf_intersect(sdf_wheat_field(global_pos), sdf_segment_border(pos, form, rotation));
    case RAIL_SEGMENT:
    case WATER_SEGMENT:
        return sdf_intersect(sdf_water(global_pos), sdf_segment_border(pos, form, rotation));
    case EMPTY_SEGMENT:
    default:
        return vec2(inf, UNDEFINED);
    }
}

vec2 sdf_tile(vec3 global_pos, vec3 pos, Tile tile) {
    vec2 scene_distance = sdf_floor(pos);
    for (int segment_index = 0; segment_index < 6; segment_index++) {
        int s = 3 * segment_index;
        if (tile.segments[s + 0] == 0) {
            break;
        }
        vec2 segment_distance = sdf_segment(global_pos, pos, tile.segments[s + 0], tile.segments[s + 1], tile.segments[s + 2]);
        scene_distance = sdf_union(scene_distance, segment_distance);
    }
    return sdf_intersect(scene_distance, sdf_tile_bounds(pos));
}

float shadow_march(Ray ray, float mint, float maxt, float w, ivec2 st, Tile tile) {
    vec3 tile_center = syt_to_xyz(st.s, 0, st.t);
    float res = 1.0;
    float previous_distance = epsilon;
    float total_distance = mint;
    for(int i=0; i < ray_march_steps && total_distance < maxt; i++) {
        vec3 global_pos = ray.origin + ray.dir * total_distance;
        vec3 relative_origin = at(global_pos, tile_center);
        float distance = sdf_tile(global_pos, relative_origin, tile).x;
        if(distance < ray_march_eps) {
            return 0.0;
        }
        float y = distance * distance / (2.0 * previous_distance);
        float d = sqrt(distance * distance - y * y);
        res = min(res, d / (w * max(0.0, total_distance - y)));
        previous_distance = distance;
        total_distance += distance;
    }
    return res;
}

/**
 * Use raymarching to intersect tile `tile` at coords `st`. The return vector
 * contains the last sdf value (proximity to any object) in the X coordinate,
 * then the total marched distance in Y, and the reported material in the Z
 * coordinate.
 */
vec3 intersect_tile(Ray ray, ivec2 st, Tile tile) {
    vec3 tile_center = syt_to_xyz(st.s, 0, st.t);
    vec3 relative_origin;
    vec2 tile_distance;
    float total_distance = 0.0;
    for (int i = 0; i < ray_march_steps; i++) {
        // Compute distance to scene.
        relative_origin = at(ray.origin, tile_center);
        tile_distance = sdf_tile(ray.origin, relative_origin, tile);

        // If we're now very close to scene.
        if (tile_distance.x < ray_march_eps) {
            return vec3(tile_distance.x, total_distance, tile_distance.y);
        }

        // Advance ray.
        total_distance += tile_distance.x;
        ray.origin += tile_distance.x * ray.dir;
    }
    return vec3(tile_distance.x, total_distance, tile_distance.y);
}

// // Rotation is CCW, but the DR rotation is CW.
// float rotation = -rot * 2 * deg_30;
// Ray rotated_ray = Ray(
//     rotate_y(at(ray.origin, tile_center), rotation),
//     rotate_y(ray.dir, rotation),
//     rotate_y(ray.inv_dir, rotation)
// );

// if(outer_terrain == WATER_SEGMENT) {
//     vec3 hit_pos = ray.origin + closest_intersection.distance * ray.dir;
//     closest_intersection.normal = rotate_x(closest_intersection.normal, 0.2 * sin(40 * hit_pos.x + 2 * time));
//     closest_intersection.normal = rotate_z(closest_intersection.normal, 0.2 * sin(40 * hit_pos.z + 2 * time));
// }

ivec2 grid_coords_at(vec3 floor) {
    // Calculate floor tile coords in skewed coordinate grid.
    int diagonal_steps = int(round(floor.z / tile_d.z));
    float vertical_offset = floor.x - diagonal_steps * tile_d.x / 2;
    int vertical_steps = int(round(vertical_offset / tile_d.x));

    // Correct edges, otherwise we'd get offset rectangles and not hexes.
    vec3 tile_center = vec3(vertical_steps * tile_d.x + diagonal_steps * tile_d.x / 2, 0, diagonal_steps * tile_d.z);
    vec3 offset_from_center = vec3(floor.x, 0, floor.z) - tile_center;

    vec3 diagonal_to_top_right = vec3(tile_d.x / 2, 0, tile_d.z) / tile_d.x;
    float offset_to_top_right = dot(offset_from_center, diagonal_to_top_right);

    if (offset_to_top_right > cos_30) {
        diagonal_steps += 1;
    } else if (offset_to_top_right < -cos_30) {
        diagonal_steps -= 1;
    }

    vec3 diagonal_to_top_left = vec3(tile_d.x / 2, 0, -tile_d.z) / tile_d.x;
    float offset_to_top_left = dot(offset_from_center, diagonal_to_top_left);

    if (offset_to_top_left > cos_30) {
        diagonal_steps -= 1;
        vertical_steps += 1;
    } else if (offset_to_top_left < -cos_30) {
        diagonal_steps += 1;
        vertical_steps -= 1;
    }

    // Convert to offset coordinate grid.
    return ivec2(
        diagonal_steps,
        vertical_steps + int(ceil((diagonal_steps - 0.5) / 2))
    );
}

vec4 intersect_scene(Ray ray) {
    ivec2 st = ivec2(10000000, 10000000);
    ivec2 next_st;

    ivec2 closest_st = st;
    vec2 closest = vec2(inf, SKYBOX);

    float dist_floor;
    vec3 floor;
    vec3 local;
    for (int i=2; i>=0; i--) {
        dist_floor = distance_to_y(ray, i * 0.2);
        if (dist_floor < 0 || dist_floor > 10000) {
            // Looking at sky.
            return vec4(inf, SKYBOX, inf, inf);
        }
        floor = ray.origin + dist_floor * ray.dir;
        next_st = grid_coords_at(floor);
        if (next_st != st) {
            st = next_st;
            Tile t = get_tile(st);
            if (t.present) {
                local = intersect_tile(ray, st, t);
                if (local.y < closest.x && local.x < 1.0) {
                    closest_st = st;
                    closest = local.yz;
                }
            }
        }
    }
    return vec4(closest, closest_st);
}

void main(){
    float aspect_ratio = float(size.s) / size.t;

    vec3 ray_origin = camera_origin;
    float fov_factor = tan(0.5 * fovy);

    vec3 ray_dir = normalize(camera_ahead + fov_factor * (uv.x * aspect_ratio * camera_right + uv.y * camera_up));

    Ray ray = Ray(ray_origin, ray_dir, 1 / ray_dir);
    vec4 intersection = intersect_scene(ray);

    float distance = intersection.x;
    float material = intersection.y;
    ivec2 st = ivec2(intersection.z, intersection.w);

    // 2 revolutions per minute.
    float light_time = 0.026 * time / (2*pi);
    vec3 light_dir = normalize(vec3(cos(light_time), 1, sin(light_time)));
    Ray light_ray = Ray(ray_origin + distance * ray_dir, light_dir, 1 / light_dir);

    Tile tile = get_tile(st);
    float shadow = shadow_march(light_ray, 0.01, 1000, 0.02, st, tile);

    frag_data = vec4(distance / 500.0, shadow, material / 100.0, 1.0);
}
