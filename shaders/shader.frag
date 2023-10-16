#version 450

in vec2 uv;

layout(location = 0) out vec4 frag_color;
layout(location = 1) out float frag_depth;

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

struct Intersection {
    float distance;
    vec3 color;
    vec3 normal;
};

struct SdfHit {
    float distance;
    vec3 color;
};

const float inf = 1.0 / 0.0;
const float pi = 3.141592653589793;
const float epsilon = 0.00001;

// PI / 6
const float deg_30 = pi * 0.166666666;
const float cos_30 = cos(deg_30);
const float sin_30 = 0.5;

const vec3 tile_d = vec3(2 * cos_30, 0, 1.5);
const Intersection no_intersection = Intersection(inf, vec3(0.0 / 0.0), vec3(0.0));

const float normal_eps = 0.004;
const vec2 normal_h = vec2(normal_eps, 0);


const int ray_march_steps = 50;
const float ray_march_eps = 0.0001;

#define normal_scene(pos_var, scene_sdf_function) normalize(vec3(   \
    scene_sdf_function(pos_var + normal_h.xyy).distance - scene_sdf_function(pos_var - normal_h.xyy).distance, \
    scene_sdf_function(pos_var + normal_h.yxy).distance - scene_sdf_function(pos_var - normal_h.yxy).distance, \
    scene_sdf_function(pos_var + normal_h.yyx).distance - scene_sdf_function(pos_var - normal_h.yyx).distance  \
))


vec3 syt_to_xyz(int s, float y, int t) {
    return vec3(t * tile_d.x - 0.5 * (s % 2) * tile_d.x, y, s * tile_d.z);
}

// // ABC is counterclockwise.
// bool intersects_parallelogram(Ray ray, vec3 a, vec3 b, vec3 c) {
//     vec3 ba = a - b;
//     vec3 bc = c - b;
//
//     vec3 pvec = cross(ray.dir, bc);
//     float det = dot(ba, pvec);
//
//     // `det == 0` means that the vectors are coplanar.
//     if (abs(det) < epsilon) {
//         return false;
//     }
//
//     float inv_det = 1 / det;
//     vec3 tvec = ray.origin - b;
//
//     float u = dot(tvec, pvec) * inv_det;
//     if (u < 0 || u > 1) {
//         return false;
//     }
//
//     vec3 qvec = cross(tvec, ba);
//     float v = dot(ray.dir, qvec) * inv_det;
//     if (v < 0 || u + v > 1) {
//         return false;
//     }
//
//     /* float t = bc.dotProduct(qvec) * inv_det; */
//
//     return true;
// }

vec2 distance_to_aabb(Ray ray, vec3 lower, vec3 upper) {
    float tx1 = (lower.x - ray.origin.x) * ray.inv_dir.x;
    float tx2 = (upper.x - ray.origin.x) * ray.inv_dir.x;
    // Max intersection with all 3 lower planes.
    float tmin = min(tx1, tx2);
    // Min intersection distance with all 3 upper planes.
    float tmax = max(tx1, tx2);

    float ty1 = (lower.y - ray.origin.y) * ray.inv_dir.y;
    float ty2 = (upper.y - ray.origin.y) * ray.inv_dir.y;
    tmin = max(tmin, min(ty1, ty2));
    tmax = min(tmax, max(ty1, ty2));

    float tz1 = (lower.z - ray.origin.z) * ray.inv_dir.z;
    float tz2 = (upper.z - ray.origin.z) * ray.inv_dir.z;
    tmin = max(tmin, min(tz1, tz2));
    tmax = min(tmax, max(tz1, tz2));

    return vec2(tmin, tmax);
}

// float distance_to_x(Ray ray, float target_x) {
//     return (target_x - ray.origin.x) * ray.inv_dir.x;
// }

float distance_to_y(Ray ray, float target_y) {
    return (target_y - ray.origin.y) * ray.inv_dir.y;
}

// float distance_to_z(Ray ray, float target_z) {
//     return (target_z - ray.origin.z) * ray.inv_dir.z;
// }

// https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-plane-and-ray-disk-intersection.html
float distance_to_plane(vec3 plane, vec3 plane_normal, Ray ray) {
    float dot_dir_normal = dot(ray.dir, plane_normal);
    if (abs(dot_dir_normal) < epsilon) {
        return inf;
    }

    vec3 ray_to_plane = plane - ray.origin;
    return dot(ray_to_plane, plane_normal) / dot_dir_normal;
}

bool intersects_aligned_hex(Ray ray, vec3 center) {
    Ray rel_ray = Ray(ray.origin - center, ray.dir, ray.inv_dir);

    float t1, t2;
    float max_near = - inf;
    float min_far = inf;

    // Flat sides of hex are facing towards +x and -x.

    // Upper and lower
    t1 = (-1.0 - rel_ray.origin.y) * rel_ray.inv_dir.y;
    t2 = (1.0 - rel_ray.origin.y) * rel_ray.inv_dir.y;
    max_near = max(max_near, min(t1, t2));
    min_far = min(min_far, max(t1, t2));

    // Front and back
    t1 = (-0.5 * tile_d.x - rel_ray.origin.x) * rel_ray.inv_dir.x;
    t2 = (0.5 * tile_d.x - rel_ray.origin.x) * rel_ray.inv_dir.x;
    max_near = max(max_near, min(t1, t2));
    min_far = min(min_far, max(t1, t2));

    // Bottom left, top right.
    vec3 plane_normal = vec3(sin_30, 0, cos_30);
    vec3 plane_pos = vec3(0, 0, 1);

    t1 = distance_to_plane(plane_pos, plane_normal, rel_ray);
    t2 = distance_to_plane(-plane_pos, -plane_normal, rel_ray);
    max_near = max(max_near, min(t1, t2));
    min_far = min(min_far, max(t1, t2));

    // Bottom right, top left.
    plane_normal = vec3(-plane_normal.x, 0, plane_normal.z);

    t1 = distance_to_plane(plane_pos, -plane_normal, rel_ray);
    t2 = distance_to_plane(-plane_pos, plane_normal, rel_ray);
    max_near = max(max_near, min(t1, t2));
    min_far = min(min_far, max(t1, t2));

    return max_near <= min_far && min_far > 0;
}

// vec3 terrain_color(int terrain) {
//     switch(terrain) {
//         case -1:
//             return vec3(0.1);
//         case 0:
//             return vec3(1, 0, 0);
//         case 1:
//             return vec3(0.6, 0.3, 0);
//         case 2:
//             return vec3(0.9, 0.8, 0);
//         case 3:
//             return vec3(0.8);
//         case 4:
//             return vec3(0, 0, 1);
//         default:
//             return vec3(1);
//     }
// }

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

vec3 repeated_x(vec3 pos, float step) {
    return vec3(pos.x - step * round(pos.x / step), pos.y, pos.z);
}

vec3 repeated_xz(vec3 pos, float step) {
    vec2 xz = pos.xz - step * round(pos.xz / step);
    return vec3(xz.x, pos.y, xz.y);
}

#define repeated_xz_variate(pos_var, index_var, step_var, scene_sdf_func, result_var) \
    vec3 index_var = round(pos_var / step_var); \
    SdfHit result_var = SdfHit(inf, vec3(0)); \
    for(int i = -1; i < 2; i++) { for(int j = -1; j < 2; j++) { \
        vec3 local_index = index_var + vec3(i, 0, j); \
        SdfHit temp = scene_sdf_func(pos_var - step_var * local_index, local_index); \
        result_var = temp.distance < result_var.distance ? temp : result_var; \
    }}

SdfHit sdf_intersect(SdfHit a, SdfHit b) {
    return SdfHit(max(a.distance, b.distance), a.color);
}

SdfHit sdf_union(SdfHit a, SdfHit b) {
    return a.distance < b.distance ? a : b;
}

SdfHit sdf_subtract(SdfHit a, SdfHit b) {
    return SdfHit(max(-a.distance, b.distance), a.color);
}

SdfHit sdf_box(vec3 pos, vec3 side_len) {
    vec3 to_corner = abs(pos) - side_len / 2;
    return SdfHit(
        length(max(to_corner, 0)) + min(maxcomp(to_corner), 0),
        vec3(1)
    );
}

SdfHit sdf_plane(vec3 pos, vec3 normal, float offset) {
    return SdfHit(
        dot(pos, normal) - offset,
        vec3(1, 0, 1)
    );
}

// /* SdfHit sdf_inverse_box(vec3 pos, vec3 size) { */
// /*     vec3 to_corner = side_len / 2 - abs(pos); */
// /*     return SdfHit(length(pos.yz) - r, vec3(1)); */
// /* } */

SdfHit sdf_sphere(vec3 pos, float r) {
    return SdfHit(length(pos) - r, vec3(1));
}

SdfHit sdf_cylinder_x(vec3 pos, float r) {
    return SdfHit(length(pos.yz) - r, vec3(1));
}

SdfHit sdf_cylinder_y(vec3 pos, float r) {
    return SdfHit(length(pos.xz) - r, vec3(1));
}

SdfHit sdf_cylinder_z(vec3 pos, float r) {
    return SdfHit(length(pos.xy) - r, vec3(1));
}

SdfHit sdf_cone_y(vec3 pos, vec2 c) {
    // c is the sin/cos of the angle
    vec2 q = vec2(length(pos.xz), -pos.y);
    float d = length(q - c * max(dot(q, c), 0.0));
    return SdfHit(d * ((q.x * c.y - q.y * c.x < 0.0) ? -1.0 : 1.0), vec3(1));
}

SdfHit sdf_base_tile(vec3 pos) {
    pos = vec3(abs(pos.x), pos.y, abs(pos.z));

    SdfHit vertical = sdf_intersect(
        SdfHit(pos.y - 0, vec3(1, 0, 1)),
        SdfHit(-pos.y - 0.1, vec3(1, 0, 1))
    );
    SdfHit front_tl = sdf_intersect(
        sdf_plane(pos, vec3(sin_30, 0, cos_30), 0.95 * cos_30),
        sdf_plane(pos, vec3(1, 0, 0), 0.95 * cos_30)
    );
    SdfHit res = sdf_intersect(vertical, front_tl);
    res.color = vec3(0.2);
    return res;
}

SdfHit sdf_roof(vec3 pos) {
    SdfHit fb = sdf_intersect(
        sdf_plane(pos, vec3(sin_30, cos_30, 0), sin_30 + 0.1),
        sdf_plane(pos, vec3(-sin_30, cos_30, 0), sin_30 + 0.1)
    );
    SdfHit lr = sdf_intersect(
        sdf_plane(pos, vec3(0, cos_30, sin_30), sin_30 + 0.1),
        sdf_plane(pos, vec3(0, cos_30, -sin_30), sin_30 + 0.1)
    );
    SdfHit bottom = SdfHit(-pos.y, vec3(1));

    return sdf_intersect(sdf_intersect(fb, lr), bottom);
}

SdfHit sdf_empty_tile(vec3 pos) {
    SdfHit tile_floor = sdf_base_tile(pos);
    tile_floor.color = vec3(0.2);
    return tile_floor;
}

// 3 in height.
// 2 in width
SdfHit sdf_house(vec3 pos) {
    SdfHit base = sdf_box(at(pos, vec3(0, 1, 0)), vec3(2));
    base.color = vec3(0.89, 0.75, 0.47);
    SdfHit roof = sdf_roof(at(pos, vec3(0, 2, 0)));
    roof.color = vec3(0.96, 0.26, 0.26);

    return sdf_union(base, roof);
}

SdfHit sdf_town(vec3 pos) {
    float scale = 0.1;
    vec3 house_offset = vec3(0.5 * cos_30, 0, 0);

    vec3 repeated_pos = repeated_xz(at(pos, house_offset) / scale, 5);
    SdfHit house = sdf_house(repeated_pos);
    house.distance *= scale;
    return house;
}

SdfHit sdf_tree(vec3 pos, vec3 index) {
    float scale = 1 + 0.2 * (abs(int(index.x) % 2) - 0.5) + 0.2 * (abs(int(index.z) % 2) - 0.5);
    scale = 1 + 0.4 * cos(index.x);
    pos /= scale;
    // pos += 0.001 * sin(index);

    SdfHit trunk_cyl = sdf_cylinder_y(pos, 0.15);
    SdfHit trunk_min = sdf_plane(pos, vec3(0, -1, 0), 0);
    SdfHit trunk_max = sdf_plane(pos, vec3(0, 1, 0), 0.5);
    SdfHit trunk = sdf_intersect(trunk_min, sdf_intersect(trunk_cyl, trunk_max));
    trunk.color = vec3(0.42, 0.35, 0.25);

    SdfHit treetop_cone = sdf_cone_y(at(pos, vec3(0, 2, 0)), vec2(sin(0.6 * deg_30), cos(0.6 * deg_30)));
    SdfHit treetop_min = sdf_plane(pos, vec3(0, -1, 0), -0.5);
    SdfHit treetop = sdf_intersect(treetop_cone, treetop_min);
    treetop.color = vec3(0.30, 0.34, 0.17);

    SdfHit result = sdf_union(trunk, treetop);
    result.distance *= scale;

    return result;
}

SdfHit sdf_forest(vec3 pos) {
    float scale = 0.2;
    pos /= scale;

    repeated_xz_variate(pos, index, vec3(1, 0, 1), sdf_tree, tree)

    tree.distance *= scale;
    return tree;

    // SdfHit horizontal = sdf_intersect(
    //     sdf_plane(pos, vec3(-sin_30, 0, cos_30), 0),
    //     sdf_plane(pos, vec3(-sin_30, 0, -cos_30), 0)
    // );
    // SdfHit front = sdf_plane(pos, vec3(1, 0, 0), 0.95 * cos_30);
    // SdfHit bounds = sdf_intersect(horizontal, front);

    // return sdf_intersect(tree, bounds);
}

SdfHit sdf_wheat_tile(vec3 pos) {
    float scale = 0.8;
    SdfHit wheat = sdf_base_tile(at(pos, vec3(0.1 * cos_30, 0.1, 0)) / scale);
    wheat.color = vec3(0.89, 0.71, 0.5);
    wheat.distance *= scale;
    return wheat;
}

SdfHit sdf_rail_tile(vec3 origin) {
    // Do everything in scaled coordinates.
    float scale = 0.1;
    vec3 scaled_pos = origin / scale;

    // Create a pair of rails.
    float rail_radius = 0.05;
    vec3 rail_offset = vec3(0, 0, 0.5);
    SdfHit rail_l = sdf_cylinder_x(at(scaled_pos, +rail_offset), rail_radius);
    SdfHit rail_r = sdf_cylinder_x(at(scaled_pos, -rail_offset), rail_radius);

    SdfHit rails = sdf_union(rail_l, rail_r);
    rails.color = vec3(0.71, 0.25, 0.05);

    // Repeat sleepers.
    vec3 pos_repeated_x = repeated_x(origin / scale, 0.5);
    SdfHit sleeper = sdf_cylinder_z(pos_repeated_x, 0.7 * rail_radius);
    sleeper.color = vec3(0.4);

    // Join stuff together.
    SdfHit rails_and_sleeper = sdf_union(rails, sleeper);

    // Size is not radius, it's the extent in that direction.
    // X is unscaled early, rest is in scaled coordinates.
    vec3 bounds_pos = vec3(0.5 * cos_30 / scale, 0, 0);
    vec3 bounds_size = vec3(cos_30 / scale, 1, 1.4);
    SdfHit rail_bounds = sdf_box(at(scaled_pos, bounds_pos), bounds_size);
    SdfHit bounded_rail = sdf_intersect(rails_and_sleeper, rail_bounds);

    // Undo scaling.
    bounded_rail.distance *= scale;
    return bounded_rail;
}

SdfHit sdf_water_tile(vec3 origin) {
    // TODO
    SdfHit tile_floor = sdf_base_tile(origin);
    tile_floor.color = vec3(0, 0, 0.6);

    return tile_floor;
}

SdfHit sdf_bahnhof_tile(vec3 pos) {
    SdfHit building = sdf_sphere(pos, 1);
    building.color = vec3(0.2, 0.3, 0.5);

    SdfHit horizontal = sdf_intersect(
        sdf_plane(pos, vec3(-sin_30, 0, cos_30), 0),
        sdf_plane(pos, vec3(-sin_30, 0, -cos_30), 0)
    );
    SdfHit front = sdf_plane(pos, vec3(1, 0, 0), 0.95 * cos_30);
    SdfHit bounds = sdf_intersect(horizontal, front);

    return sdf_intersect(bounds, building);
}

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

SdfHit sdf_segment(vec3 global_pos, vec3 pos, int form, int terrain, int rotation) {
    switch(terrain) {
    case HOUSE_SEGMENT:
        return sdf_town(global_pos);
    case FOREST_SEGMENT:
        return sdf_forest(global_pos);
    case WHEAT_SEGMENT:
        break;
    case RAIL_SEGMENT:
        break;
    case WATER_SEGMENT:
        break;
    case EMPTY_SEGMENT:
    default:
        return SdfHit(inf, vec3(0));
    }
}

Intersection intersect_tile(Ray ray, ivec2 st, Tile tile) {
    vec3 tile_center = syt_to_xyz(st.s, 0, st.t);
    float distance = 0.0;
    for (int i = 0; i < ray_march_steps; i++) {
        vec3 relative_origin = at(ray.origin, tile_center);
        SdfHit scene = sdf_base_tile(relative_origin);
        for (int segment_index = 0; segment_index < 6; segment_index++) {
            int s = 3 * segment_index;
            if (tile.segments[s + 0] == 0) {
                break;
            }
            SdfHit segment = sdf_segment(ray.origin, relative_origin, tile.segments[s + 0], tile.segments[s + 1], tile.segments[s + 2]);
            scene = sdf_union(scene, segment);
            if (scene.distance < ray_march_eps) {
                return Intersection(distance, scene.color, vec3(0, 1, 0));
            }
        }

        distance += scene.distance;
        ray.origin += scene.distance * ray.dir;
    }
    return no_intersection;
}

// int outer_terrain = tile.segments[rot];
// if (tile.special_id == 1) {
//     object_intersection = intersect_bahnhof(rotated_ray);
// } else {
//     // Place things according to the type of the segment.
//     // Compute the center position of each segment first.
//     if (outer_terrain == EMPTY_SEGMENT) {
//         object_intersection = intersect_empty(rotated_ray);
//     } else if (outer_terrain == HOUSE_SEGMENT) {
//         object_intersection = intersect_house(rotated_ray);
//     } else if(outer_terrain == FOREST_SEGMENT) {
//         object_intersection = intersect_forest(rotated_ray);
//     } else if(outer_terrain == WHEAT_SEGMENT) {
//         object_intersection = intersect_wheat(rotated_ray);
//     } else if(outer_terrain == RAIL_SEGMENT) {
//         object_intersection = intersect_rail(rotated_ray);
//     } else if(outer_terrain == WATER_SEGMENT) {
//         object_intersection = intersect_water(rotated_ray);
//     }
// }

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

Intersection intersect_scene(Ray ray) {
    float dist_floor = distance_to_y(ray, 0.0);
    if (dist_floor < 0 || dist_floor > 10000) {
        // Looking at sky.
        return Intersection(inf, vec3(0.6, 0.6, 0.9), vec3(0, 1, 0));
    }
    vec3 floor = ray.origin + dist_floor * ray.dir;

    // Calculate floor tile coords in skewed coordinate grid.
    int diagonal_steps = int(round(floor.z / tile_d.z));
    float vertical_offset = floor.x - diagonal_steps * tile_d.x / 2;
    int vertical_steps = int(round(vertical_offset / tile_d.x));

    // Correct edges, otherwise we'd get offset rectangles and not hexes.
    vec3 tile_center = vec3(vertical_steps * tile_d.x + diagonal_steps * tile_d.x / 2, 0, diagonal_steps * tile_d.z);
    vec3 offset_from_center = floor - tile_center;

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
    ivec2 st = ivec2(
        diagonal_steps,
        vertical_steps + int(ceil((diagonal_steps - 0.5) / 2))
    );

    Tile t = get_tile(st);
    if (t.present) {
        return intersect_tile(ray, st, t);
    }

    return Intersection(1.0 / 0.0, vec3(1.0, 0.0, 1.0), vec3(0));
}

void main(){
    float aspect_ratio = float(size.s) / size.t;

    vec3 ray_origin = camera_origin;
    float fov_factor = tan(0.5 * fovy);

    vec3 ray_dir = normalize(camera_ahead + fov_factor * (uv.x * aspect_ratio * camera_right + uv.y * camera_up));

    Ray ray = Ray(ray_origin, ray_dir, 1 / ray_dir);
    Intersection intersection = intersect_scene(ray);
    frag_color = vec4(intersection.color, 1);
    frag_depth = 1.0;

    // vec3 ambient = 0.1 * intersection.color;
    // vec3 light = vec3(0.577);
    // vec3 diffuse = vec3(max(2.0 * dot(intersection.normal, light), 0.0));
    // vec3 half_light_view = normalize(light - ray.dir);
    // vec3 specular = vec3(max(1.0 * pow(dot(intersection.normal, half_light_view), 200.0), 0.0));
    // vec3 lightness = ambient + diffuse + specular;

    // frag_color = vec4(intersection.color * lightness, 1);
}
