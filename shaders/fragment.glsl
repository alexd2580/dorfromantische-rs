#version 450

in vec2 uv;
out vec4 frag_color;

layout (std140) uniform globals_buffer {
    float time;
};

layout (std140) uniform view_buffer {
    int width;
    int height;

    float fovy;
    float _pad;

    vec3 camera_origin;
    vec3 camera_right;
    vec3 camera_ahead;
    vec3 camera_up;
};

layout (std140) uniform chunk_meta_buffer {
    int chunk_size;
    int num_chunk_items;
    int num_chunk_levels;
    int num_level0_chunks;
};

struct Tile {
    int s;
    int t;
    int segments[6];
};

#define EMPTY_SEGMENT -1
#define HOUSE_SEGMENT 0
#define FOREST_SEGMENT 1
#define WHEAT_SEGMENT 2
#define RAIL_SEGMENT 3
#define WATER_SEGMENT 4

layout(std430) buffer tiles_buffer {
    Tile tiles[];
};

struct ChunkHeader {
    int s;
    int t;
};

layout(std430) buffer chunk_headers_buffer {
    ChunkHeader chunk_headers[];
};

layout(std430) buffer chunk_indices_buffer {
    int chunk_indices[];
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

const float pi = 3.141592653589793;
const float epsilon = 0.00001;
// PI / 6
const float deg_30 = pi * 0.166666666;
const float cos_30 = cos(deg_30);
const float sin_30 = 0.5;

const vec3 tile_d = vec3(2 * cos_30, 0, 1.5);
const Intersection no_intersection = Intersection(1.0 / 0.0, vec3(0.0 / 0.0), vec3(0.0));

const float normal_eps = 0.004;
const vec2 normal_h = vec2(normal_eps, 0);

const float ray_march_eps = 0.001;

#define normal_scene(pos_var, scene_sdf_function) normalize(vec3(   \
    scene_sdf_function(pos_var + normal_h.xyy).distance - scene_sdf_function(pos_var - normal_h.xyy).distance, \
    scene_sdf_function(pos_var + normal_h.yxy).distance - scene_sdf_function(pos_var - normal_h.yxy).distance, \
    scene_sdf_function(pos_var + normal_h.yyx).distance - scene_sdf_function(pos_var - normal_h.yyx).distance  \
))


vec3 syt_to_xyz(int s, float y, int t) {
    return vec3(t * tile_d.x - 0.5 * (s % 2) * tile_d.x, y, s * tile_d.z);
}

// ABC is counterclockwise.
bool intersects_parallelogram(Ray ray, vec3 a, vec3 b, vec3 c) {
    vec3 ba = a - b;
    vec3 bc = c - b;

    vec3 pvec = cross(ray.dir, bc);
    float det = dot(ba, pvec);

    // `det == 0` means that the vectors are coplanar.
    if (abs(det) < epsilon) {
        return false;
    }

    float inv_det = 1 / det;
    vec3 tvec = ray.origin - b;

    float u = dot(tvec, pvec) * inv_det;
    if (u < 0 || u > 1) {
        return false;
    }

    vec3 qvec = cross(tvec, ba);
    float v = dot(ray.dir, qvec) * inv_det;
    if (v < 0 || u + v > 1) {
        return false;
    }

    /* float t = bc.dotProduct(qvec) * inv_det; */

    return true;
}

float distance_to_aabb(Ray ray, vec3 lower, vec3 upper) {
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

    return tmin <= tmax && tmax > 0 ? max(tmin, 0) : 1.0 / 0.0;
}

// https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-plane-and-ray-disk-intersection.html
float distance_to_plane(vec3 plane, vec3 plane_normal, Ray ray) {
    float dot_dir_normal = dot(ray.dir, plane_normal);
    if (abs(dot_dir_normal) < epsilon) {
        return 1.0 / 0.0;
    }

    vec3 ray_to_plane = plane - ray.origin;
    return dot(ray_to_plane, plane_normal) / dot_dir_normal;
}

bool intersects_aligned_hex(Ray ray, int s, int t) {
    vec3 lower = syt_to_xyz(s, 0, t);
    Ray rel_ray = Ray(ray.origin - lower, ray.dir, ray.inv_dir);

    float t1, t2;
    float max_near = - 1.0 / 0.0;
    float min_far = 1.0 / 0.0;

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

vec3 terrain_color(int terrain) {
    switch(terrain) {
        case -1:
            return vec3(0.1);
        case 0:
            return vec3(1, 0, 0);
        case 1:
            return vec3(0.6, 0.3, 0);
        case 2:
            return vec3(0.9, 0.8, 0);
        case 3:
            return vec3(0.8);
        case 4:
            return vec3(0, 0, 1);
        default:
            return vec3(1);
    }
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

vec3 repeated_x(vec3 pos, float step) {
    return vec3(mod(pos.x - 0.5 * step, step) * step, pos.y, pos.z);
}

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
        vec3(1)
    );
}

/* SdfHit sdf_inverse_box(vec3 pos, vec3 size) { */
/*     vec3 to_corner = side_len / 2 - abs(pos); */
/*     return SdfHit(length(pos.yz) - r, vec3(1)); */
/* } */

SdfHit sdf_cylinder_x(vec3 pos, float r) {
    return SdfHit(length(pos.yz) - r, vec3(1));
}

SdfHit sdf_cylinder_z(vec3 pos, float r) {
    return SdfHit(length(pos.xy) - r, vec3(1));
}

SdfHit sdf_base_tile(vec3 pos) {
    SdfHit vertical = sdf_intersect(
        SdfHit(pos.y - 0, vec3(1)),
        SdfHit(-pos.y - 0.1, vec3(1))
    );
    SdfHit horizontal = sdf_intersect(
        sdf_plane(pos, vec3(-sin_30, 0, cos_30), 0),
        sdf_plane(pos, vec3(-sin_30, 0, -cos_30), 0)
    );
    SdfHit front = sdf_plane(pos, vec3(1, 0, 0), cos_30);

    return sdf_intersect(sdf_intersect(vertical, horizontal), front);
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

Intersection intersect_empty(Ray ray) {
    int num_steps = 50;
    float distance = 0;
    for (int i = 0; i < num_steps; i++) {
        SdfHit scene = sdf_empty_tile(ray.origin);

        if (scene.distance < ray_march_eps) {
            return Intersection(distance, scene.color, normal_scene(ray.origin, sdf_empty_tile));
        }

        distance += scene.distance;
        ray.origin += scene.distance * ray.dir;
    }
    return no_intersection;
}

// 3 in height.
// 2 in width
SdfHit sdf_house(vec3 pos) {
    SdfHit base = sdf_box(at(pos, vec3(0, 1, 0)), vec3(2));
    base.color = vec3(0.5);
    SdfHit roof = sdf_roof(at(pos, vec3(0, 2, 0)));
    roof.color = vec3(0.6, 0, 0);

    return sdf_union(base, roof);
}

SdfHit sdf_house_tile(vec3 pos) {
    SdfHit tile_floor = sdf_base_tile(pos);
    tile_floor.color = vec3(0.2);

    float scale = 0.1;
    vec3 house_offset = vec3(0.5 * cos_30, 0, 0);

    SdfHit house = sdf_house(at(pos, house_offset) / scale);
    house.distance *= scale;

    return sdf_union(tile_floor, house);
}

Intersection intersect_house(Ray ray) {
    int num_steps = 50;
    float distance = 0;
    for (int i = 0; i < num_steps; i++) {
        SdfHit scene = sdf_house_tile(ray.origin);

        if (scene.distance < ray_march_eps) {
            return Intersection(distance, scene.color, normal_scene(ray.origin, sdf_house_tile));
        }

        distance += scene.distance;
        ray.origin += scene.distance * ray.dir;
    }
    return no_intersection;
}

SdfHit sdf_wheat_tile(vec3 pos) {
    SdfHit tile_floor = sdf_base_tile(pos);
    tile_floor.color = vec3(0.2);

    float scale = 0.8;
    SdfHit wheat = sdf_base_tile(at(pos, vec3(0.1 * cos_30, 0.1, 0)) / scale);
    wheat.color = vec3(0.89, 0.71, 0.5);
    wheat.distance *= scale;

    return sdf_union(tile_floor, wheat);
}

Intersection intersect_wheat(Ray ray) {
    int num_steps = 50;
    float distance = 0;
    for (int i = 0; i < num_steps; i++) {
        SdfHit scene = sdf_wheat_tile(ray.origin);

        if (scene.distance < ray_march_eps) {
            return Intersection(distance, scene.color, normal_scene(ray.origin, sdf_wheat_tile));
        }

        distance += scene.distance;
        ray.origin += scene.distance * ray.dir;
    }
    return no_intersection;
}

SdfHit sdf_rail_tile(vec3 origin) {
    // Do everything in scaled coordinates.
    float scale = 0.1;
    vec3 scaled_pos = origin / scale;

    // Create a pair of rails.
    vec3 rail_v_offset = vec3(0, 0.03, 0);
    vec3 rail_offset = vec3(0, 0, 0.5);
    SdfHit rail_l = sdf_cylinder_x(at(scaled_pos, rail_v_offset + rail_offset), 0.05);
    SdfHit rail_r = sdf_cylinder_x(at(scaled_pos, rail_v_offset - rail_offset), 0.05);

    SdfHit rails = sdf_union(rail_l, rail_r);
    rails.color = vec3(0.71, 0.25, 0.05);

    // Repeat sleepers.
    vec3 pos_repeated_x = repeated_x(origin / scale, 1);
    SdfHit sleeper = sdf_cylinder_z(pos_repeated_x, 0.05);
    sleeper.color = vec3(0.82);

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

    // Add floor.
    SdfHit tile_floor = sdf_base_tile(origin);
    tile_floor.color = vec3(0.2);
    return sdf_union(tile_floor, bounded_rail);
}

Intersection intersect_rail(Ray ray) {
    int num_steps = 50;
    float distance = 0;
    for (int i = 0; i < num_steps; i++) {
        SdfHit scene = sdf_rail_tile(ray.origin);

        if (scene.distance < ray_march_eps) {
            return Intersection(distance, scene.color, normal_scene(ray.origin, sdf_rail_tile));
        }

        distance += scene.distance;
        ray.origin += scene.distance * ray.dir;
    }
    return no_intersection;
}

SdfHit sdf_water_tile(vec3 origin) {
    SdfHit tile_floor = sdf_base_tile(origin);
    tile_floor.color = vec3(0, 0, 0.6);

    return tile_floor;
}

Intersection intersect_water(Ray ray) {
    int num_steps = 50;
    float distance = 0;
    for (int i = 0; i < num_steps; i++) {
        SdfHit scene = sdf_water_tile(ray.origin);

        if (scene.distance < ray_march_eps) {
            vec3 normal = normal_scene(ray.origin, sdf_water_tile);

            // 0.2 = PI / 16
            return Intersection(distance, scene.color, normal);
        }

        distance += scene.distance;
        ray.origin += scene.distance * ray.dir;
    }
    return no_intersection;
}

Intersection intersect_tile(Ray ray, int tile_index, Tile tile) {
    if (!intersects_aligned_hex(ray, tile.s, tile.t)) {
        return no_intersection;
    }

    vec3 tile_center = syt_to_xyz(tile.s, 0, tile.t);

    // The ray intersects the hex BB.
    // Place all items into the scene and intersect with them.
    Intersection closest_intersection = no_intersection;
    Intersection object_intersection = no_intersection;

    for (int rot = 0; rot < 6; rot++) {
        // Roration is CCW, but the DR rotation is CW.
        float rotation = -rot * 2 * deg_30;
        Ray rotated_ray = Ray(
            rotate_y(at(ray.origin, tile_center), rotation),
            rotate_y(ray.dir, rotation),
            rotate_y(ray.inv_dir, rotation)
        );

        // Place things according to the type of the segment.
        // Compute the center position of each segment first.
        int outer_terrain = tile.segments[rot];
        if (outer_terrain == EMPTY_SEGMENT) {
            object_intersection = intersect_empty(rotated_ray);
        } else if (outer_terrain == HOUSE_SEGMENT) {
            object_intersection = intersect_house(rotated_ray);
        /* } else if(outer_terrain == FOREST_SEGMENT) { */
        /*     object_intersection = intersect_forest(rotated_ray); */
        } else if(outer_terrain == WHEAT_SEGMENT) {
            object_intersection = intersect_wheat(rotated_ray);
        } else if(outer_terrain == RAIL_SEGMENT) {
            object_intersection = intersect_rail(rotated_ray);
        } else if(outer_terrain == WATER_SEGMENT) {
            object_intersection = intersect_water(rotated_ray);
        }

        if (object_intersection.distance < closest_intersection.distance) {
            closest_intersection = object_intersection;
            closest_intersection.normal = rotate_y(closest_intersection.normal, -rotation);

            // Add some waves to the water using normals.
            if(outer_terrain == WATER_SEGMENT) {
                vec3 hit_pos = ray.origin + closest_intersection.distance * ray.dir;
                closest_intersection.normal = rotate_x(closest_intersection.normal, 0.2 * sin(40 * hit_pos.x + 2 * time));
                closest_intersection.normal = rotate_z(closest_intersection.normal, 0.2 * sin(40 * hit_pos.z + 2 * time));
            }
        }

        /* float center_offset = sqrt(dot(p, p)); */
        /* float angle = acos(p.z / center_offset); */
        /* bool below_z = p.x < 0; */
        /* angle = below_z ? 2 * PI - angle : angle; */
        /* // DR indexes clockwise from 1200. */
        /* int rot = (7 - int(floor(angle * 3 / PI))) % 6; */
    }

    return closest_intersection;

    /* // INACCURATE FOR PRECISE STUFF! */
    /* float distance_bot = (0.0 - ray.origin.y) * ray.inv_dir.y; */
    /* // TODO volume */
    /* float distance_top = (0.0 - ray.origin.y) * ray.inv_dir.y; */
    /* float distance = min(distance_bot, distance_top); */
    /* if (distance < 0) { */
    /*     return no_intersection; */
    /* } */
    /*  */
    /* vec3 xoz_intersection = ray.origin + distance * ray.dir; */
    /* vec3 tile_center = syt_to_xyz(tile.s, 0.0, tile.t); */
    /* vec3 p = xoz_intersection - tile_center; */
    /*  */
    /* float x_offset = abs(p.x); */
    /* float xz_offset = abs(p.z) + abs(p.x) / tile_d.x; */
    /*  */
    /* if (x_offset > 0.9 * 0.5 * tile_d.x || xz_offset > 0.9 * 1) { */
    /*     return no_intersection; */
    /* } */
    /*  */
    /* float center_offset = sqrt(dot(p, p)); */
    /* float angle = acos(p.z / center_offset); */
    /* bool below_z = p.x < 0; */
    /* angle = below_z ? 2 * PI - angle : angle; */
    /*  */
    /* // We compute the index counterclockwise from 0300. */
    /* // DR indexes clockwise from 1200. */
    /* int rot = (7 - int(floor(angle * 3 / PI))) % 6; */
    /*  */
    /* int inner_terrain = tile.inner_segments[rot]; */
    /* int outer_terrain = tile.outer_segments[rot]; */
    /*  */
    /* bool is_inner = x_offset < 0.45 * 0.5 * tile_d.x && xz_offset < 0.45 * 1; */
    /* int terrain = is_inner && inner_terrain != -1 ? inner_terrain : outer_terrain; */
    /* return Intersection(true, distance, terrain_color(terrain)); */
}

Intersection intersect_chunk_tiles(Ray ray, int chunk_index) {
    Intersection closest_intersection = no_intersection;
    Intersection tile_intersection;

    int read_offset = chunk_index * num_chunk_items;

    // For every tile of this chunk...
    for (int i = 0; i < num_chunk_items; i++) {
        int tile_index = chunk_indices[read_offset + i];
        if (tile_index == -1) {
            continue;
        }

        tile_intersection = intersect_tile(ray, tile_index, tiles[tile_index]);
        if (tile_intersection.distance < closest_intersection.distance) {
            closest_intersection = tile_intersection;
        }
    }
    return closest_intersection;
}

// Look in the headers to check whether the ray intersects the aabb of this chunk.
float distance_to_chunk(Ray ray, int chunk_index, int chunk_level) {
    // Get min and max indices.
    int s1 = chunk_headers[chunk_index].s, t1 = chunk_headers[chunk_index].t;

    // Could be broken, off by one.
    int chunk_width = int(pow(chunk_size, pow(2, num_chunk_levels - chunk_level - 1)));

    int s2 = s1 + chunk_width - 1, t2 = t1 + chunk_width - 1;

    // `tile_d` is slightly larger than needed.
    vec3 lower = syt_to_xyz(s1, -1.0, t1) - tile_d;
    vec3 upper = syt_to_xyz(s2, 1.0, t2) + tile_d;

    return distance_to_aabb(ray, lower, upper);
}

Intersection intersect_scene(Ray ray) {
    Intersection closest_intersection = no_intersection;
    float chunk_distance;
    Intersection chunk_intersection;

    int chunk_level = 0;
    int chunk_level_indices_offset[8];
    int chunk_level_read_pos[8];

    // Read num_level0_chunks headers at pos 0. The initial loop is weird...
    for (int level0_index = 0; level0_index < num_level0_chunks; level0_index++) {
        if (isinf(distance_to_chunk(ray, level0_index, 0))) {
            continue;
        }

        chunk_level = 1;
        chunk_level_indices_offset[chunk_level] = level0_index * num_chunk_items;
        chunk_level_read_pos[chunk_level] = 0;

        // Return to for loop when level 0.
        while(chunk_level > 0) {
            // IF the current tile is off-chunk, move up.
            int pos_within_chunk = chunk_level_read_pos[chunk_level];
            if (pos_within_chunk == num_chunk_items) {
                chunk_level--;
                continue;
            }

            // Otherwise check this chunk POSITION WHICH IS NOT THE INDEX.
            int chunk_pos = chunk_level_indices_offset[chunk_level] + pos_within_chunk;
            // Now that we've read the chunk read pos (and are about to check
            // it once), we can move the next ptr to the next item.
            chunk_level_read_pos[chunk_level] += 1;

            // Get the actual index and check it.
            int chunk_index = chunk_indices[chunk_pos];
            if (chunk_index == -1) {
                continue;
            }

            chunk_distance = distance_to_chunk(ray, chunk_index, chunk_level);
            if (isinf(chunk_distance) || chunk_distance > closest_intersection.distance) {
                continue;
            }

            // Wenn es das letzte chunk level ist, dann kommen jetzt tile indices, andere funktion.
            if (chunk_level == num_chunk_levels - 1) {
                // No see if we got something closer than before.
                chunk_intersection = intersect_chunk_tiles(ray, chunk_index);
                if(chunk_intersection.distance < closest_intersection.distance) {
                    closest_intersection = chunk_intersection;
                }
            } else {
                chunk_level++;
                chunk_level_indices_offset[chunk_level] = chunk_index * num_chunk_items;
                chunk_level_read_pos[chunk_level] = 0;
            }

            // We've already moved the pointers further, no manual actions.
        }
    }
    // Nothign matches at all
    return closest_intersection;
}

// vec3 origin_to_center = tile_pos - ray_origin;
// float center_projection_onto_dir = dot(origin_to_center, ray_dir);
// vec3 flat_hit_point = ray_origin + center_projection_onto_dir * ray_dir;
// vec3 flat_hit_to_center = flat_hit_point - tile_pos;
// if (dot(flat_hit_to_center, flat_hit_to_center) < 1) {
//     return vec3(1, 0, 0);
// }

void main(){
    float aspect_ratio = float(width) / height;

    vec3 ray_origin = camera_origin;
    float fov_factor = tan(0.5 * fovy);

    vec3 ray_dir = normalize(camera_ahead + fov_factor * (uv.x * aspect_ratio * camera_right + uv.y * camera_up));

    Ray ray = Ray(ray_origin, ray_dir, 1 / ray_dir);
    Intersection intersection = intersect_scene(ray);

    vec3 ambient = 0.1 * intersection.color;
    vec3 light = vec3(0.577);
    vec3 diffuse = vec3(max(2.0 * dot(intersection.normal, light), 0.0));
    vec3 half_light_view = normalize(light - ray.dir);
    vec3 specular = vec3(max(1.0 * pow(dot(intersection.normal, half_light_view), 200.0), 0.0));
    vec3 lightness = ambient + diffuse + specular;

    frag_color = vec4(intersection.color * lightness, 1);

    // frag_color = vec4(vec2(width / 1500.0, height / 1500.0), 0, 1);
    // frag_color = vec4(ray_dir, 1);
}
