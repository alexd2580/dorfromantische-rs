#version 450

in vec2 uv;
out vec4 frag_color;

layout (std140) uniform globals_buffer {
    float time;
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

layout (std430) buffer quadtree_buffer {
    ivec2 qt_root_pos;
    int qt_root_width;
    int qt_pad;
    int[4] qt_nodes[];
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

float distance_to_x(Ray ray, float target_x) {
    return (target_x - ray.origin.x) * ray.inv_dir.x;
}
float distance_to_z(Ray ray, float target_z) {
    return (target_z - ray.origin.z) * ray.inv_dir.z;
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

bool intersects_aligned_hex(Ray ray, vec3 center) {
    Ray rel_ray = Ray(ray.origin - center, ray.dir, ray.inv_dir);

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

Intersection intersect_tile(Ray ray, int tile_index) {
    Tile tile = tiles[tile_index];
    vec3 tile_center = syt_to_xyz(tile.s, 0, tile.t);

    if (!intersects_aligned_hex(ray, tile_center)) {
        return no_intersection;
    }

    return Intersection(0, vec3(tile_center) / 100, vec3(0));

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
    }

    return closest_intersection;
}

struct Vars {
    int node_id;
    ivec2 st;
    int step_size;
    int child_index;

    float near;
    float far;
};

// RETURNS Z in Y POSITION!!!
vec2 st_to_xz(ivec2 st) {
    return vec2(st.t * tile_d.x - 0.5 * (st.s % 2) * tile_d.x, st.s * tile_d.z);
}

const int leaf_bit = 1 << 31;

Intersection intersect_scene(Ray ray) {
    Intersection closest_intersection = no_intersection;
    Intersection tile_intersection;

    int stack_depth = 0;
    Vars stack[20];

    stack[0].node_id = 0;
    stack[0].st = qt_root_pos;
    stack[0].step_size = qt_root_width >> 1;
    stack[0].child_index = 0;

    ivec2 lower_st;
    ivec2 upper_st = stack[0].st + ivec2(qt_root_width - 1);
    vec3 lower = syt_to_xyz(stack[0].st.s, -1.0, stack[0].st.t) - tile_d;
    vec3 upper = syt_to_xyz(upper_st.s, 1.0, upper_st.t) + tile_d;
    vec2 aabb_dist = distance_to_aabb(ray, lower, upper);

    stack[0].near = aabb_dist.x;
    stack[0].far = aabb_dist.y;

    float quad_near;
    float quad_far;

    while(stack_depth >= 0) {
        // Test the specified child of the current node.
        int parent_id = stack[stack_depth].node_id;

        // Automatically post-increment the index.
        // Primes the next child node to be checked.
        int child_index = stack[stack_depth].child_index++;

        // After checking all 4 children of a node move up a level.
        if (child_index == 4) {
            stack_depth--;
            continue;
        }

        int child_id = qt_nodes[parent_id][child_index];
        if (child_id == -1) {
            // No branch here.
            continue;
        }

        // Entering the leaf checking here, without a prior AABB check for a single tile.
        // TODO BENCHMARK!
        // Check if the next node is a leaf, or another node.
        if (bool(child_id & leaf_bit)) {
            tile_intersection = intersect_tile(ray, child_id & ~leaf_bit);
            if (tile_intersection.distance < closest_intersection.distance) {
                closest_intersection = tile_intersection;
            }
            continue;
        }

        bool is_upper_xt = child_index > 1;
        bool is_upper_zs = bool(child_index & 1);

        // This is the lower bound (of the upper quadrant).
        // Add (cos_30, 0, 1) if lower quadrant because lower quadrant has "further" upper bounds.
        // Add (0, 0, 0.5) if upper quadrant.
        ivec2 lower_end_st = stack[stack_depth].st + ivec2(stack[stack_depth].step_size - 1);
        vec2 mid_point = st_to_xz(lower_end_st);
        mid_point + vec2(is_upper_xt ? 0 : cos_30, is_upper_zs ? 0.5 : 1.0);

        // Reset local bounds.
        quad_near = stack[stack_depth].near;
        quad_far = stack[stack_depth].far;

        // X/T Direction.
        float dist_x = distance_to_x(ray, mid_point.x);
        if (is_upper_xt == ray.dir.x > 0) {
            // Midpoint is near, offset near to max.
            quad_near = max(quad_near, dist_x);
        } else {
            // Midpoint is far side, so far side comes closer.
            quad_far = min(quad_far, dist_x);
        }

        // Z/S Direction.
        float dist_z = distance_to_z(ray, mid_point.y); // Yes, y is correct here.
        if (is_upper_zs == ray.dir.z > 0) {
            quad_near = max(quad_near, dist_z);
        } else {
            quad_far = min(quad_far, dist_z);
        }

        // If the first point of entry of all halfspaces is post the first exit
        // point, then there is no hit. If the earliest exit point is behind me
        // -||-. If the current AABB near point is farther than the closest
        // intersection, then there's no point in checking.
        if (quad_near > closest_intersection.distance) {
            continue;
        }
        if (quad_near > quad_far) {
            continue;
        }
        if (quad_far < 0) {
            continue;
        }

        stack_depth++;

        if (stack_depth >= 11) {
            return Intersection(0, vec3(1, 0, 1), vec3(0));
        }

        stack[stack_depth].node_id = child_id;
        stack[stack_depth].st = stack[stack_depth - 1].st + stack[stack_depth - 1].step_size * ivec2(is_upper_zs, is_upper_xt);
        stack[stack_depth].step_size = stack[stack_depth - 1].step_size / 2;
        stack[stack_depth].child_index = 0;
        stack[stack_depth].near = quad_near;
        stack[stack_depth].far = quad_far;
    }

    return closest_intersection;
}

void main(){
    float aspect_ratio = float(size.s) / size.t;

    vec3 ray_origin = camera_origin;
    float fov_factor = tan(0.5 * fovy);

    vec3 ray_dir = normalize(camera_ahead + fov_factor * (uv.x * aspect_ratio * camera_right + uv.y * camera_up));

    Ray ray = Ray(ray_origin, ray_dir, 1 / ray_dir);
    Intersection intersection = intersect_scene(ray);
    frag_color = vec4(intersection.color, 1);
    return;

    vec3 ambient = 0.1 * intersection.color;
    vec3 light = vec3(0.577);
    vec3 diffuse = vec3(max(2.0 * dot(intersection.normal, light), 0.0));
    vec3 half_light_view = normalize(light - ray.dir);
    vec3 specular = vec3(max(1.0 * pow(dot(intersection.normal, half_light_view), 200.0), 0.0));
    vec3 lightness = ambient + diffuse + specular;

    frag_color = vec4(intersection.color * lightness, 1);
}
