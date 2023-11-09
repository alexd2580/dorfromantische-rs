use std::f32::consts::FRAC_PI_2;
use std::time::SystemTime;
use std::{collections::HashSet, fs::File, rc::Rc};

use compute_shade_rs::event_loop::{ControlFlow, Event};
use compute_shade_rs::vulkan::resources::buffer::BufferUsage;
use compute_shade_rs::winit::event::{ElementState, MouseButton, VirtualKeyCode};
use compute_shade_rs::{error, event_loop, vk, vulkan, window};
use glam::{Quat, Vec3, Vec3A};

use crate::data::{self, SaveGame};

const PAD_: usize = 4;
const BOOL_: usize = 4;
const INT_: usize = 4;
const FLOAT_: usize = 4;
const VEC3_: usize = 3 * FLOAT_ + PAD_;
const IVEC4_: usize = 4 * INT_;
const TILE_: usize = BOOL_ + INT_ + 18 * INT_ + 4 * INT_;

#[derive(Clone, Copy)]
enum Form {
    Size1 = 0,
    Size2 = 1,
    Bridge = 2,   // 1-skip1-1
    Straight = 3, // 1-skip2-1
    Size3 = 4,
    JunctionLeft = 5,  // 2-skip1-1
    JunctionRight = 6, // 2-skip2-1
    ThreeWay = 7,      // 1-skip1-1-skip1-1
    Size4 = 8,
    FanOut = 9, // 3-skip1-1
    X = 10,     // 2-skip1-2
    Size5 = 11,
    Size6 = 12,

    Unknown102 = 14,
    Unknown105 = 15,
    WaterSize4 = 16, // wtf?
    Unknown111 = 17,
}

impl From<&data::SegmentTypeId> for Form {
    fn from(value: &data::SegmentTypeId) -> Self {
        match value.0 {
            1 => Form::Size1,
            2 => Form::Size2,
            3 => Form::Bridge,
            4 => Form::Straight,
            5 => Form::Size3,
            6 => Form::JunctionLeft,
            7 => Form::JunctionRight,
            8 => Form::ThreeWay,
            9 => Form::Size4,
            10 => Form::FanOut,
            11 => Form::X,
            12 => Form::Size5,
            13 => Form::Size6,
            102 => Form::Unknown102,
            105 => Form::Unknown105,
            109 => Form::WaterSize4,
            111 => Form::Unknown111,
            other => panic!("Unexpected segment type value {other}"),
        }
    }
}

#[derive(Clone, Copy)]
enum Terrain {
    Empty = 0,
    House = 1,
    Forest = 2,
    Wheat = 3,
    Rail = 4,
    Water = 5,
}

impl From<&data::GroupTypeId> for Terrain {
    fn from(value: &data::GroupTypeId) -> Self {
        match value.0 {
            -1 => Terrain::Empty,
            0 => Terrain::House,
            1 => Terrain::Forest,
            2 => Terrain::Wheat,
            3 => Terrain::Rail,
            4 => Terrain::Water,
            other => panic!("Unexpected terrain type value {other}"),
        }
    }
}

struct Segment {
    form: Form,
    terrain: Terrain,
    rotation: i32,
    group: usize,
}

impl From<&data::Segment> for Segment {
    fn from(value: &data::Segment) -> Self {
        Self {
            form: (&value.segment_type).into(),
            terrain: (&value.group_type).into(),
            rotation: value.rotation,
            group: 0,
        }
    }
}

struct Tile {
    s: i32,
    t: i32,
    special: i32,
    segments: Vec<Segment>,
}

impl From<&data::Tile> for Tile {
    fn from(value: &data::Tile) -> Self {
        let segments = value
            .segments
            .iter()
            .map(|segment| {
                let mut segment = Segment::from(segment);
                segment.rotation += value.rotation;
                segment
            })
            .collect();

        Self {
            s: value.s,
            t: value.t,
            special: value.special_tile_id.0,
            segments,
        }
    }
}

impl Tile {
    fn quadrant_of(s: i32, t: i32) -> usize {
        match (s >= 0, t >= 0) {
            (true, true) => 0,
            (false, true) => 1,
            (false, false) => 2,
            (true, false) => 3,
        }
    }

    fn quadrant(&self) -> usize {
        Tile::quadrant_of(self.s, self.t)
    }

    fn index_of(s: i32, t: i32) -> usize {
        let s_ = if s >= 0 { s } else { -1 - s };
        let t_ = if t >= 0 { t } else { -1 - t };
        let st = s_ + t_;
        (((st + 1) * st / 2) + t_).try_into().unwrap()
    }

    fn index(&self) -> usize {
        Tile::index_of(self.s, self.t)
    }
}

pub struct App {
    savegame: SaveGame,
    tiles: Vec<Tile>,
    quadrants: [Vec<Option<usize>>; 4],

    program_start: SystemTime,

    globals_buffer: Rc<vulkan::multi_buffer::MultiBuffer>,
    view_buffer: Rc<vulkan::multi_buffer::MultiBuffer>,
    quadrant_buffers: [Rc<vulkan::multi_buffer::MultiBuffer>; 4],

    data_image: Rc<vulkan::multi_image::MultiImage>,
    vulkan: vulkan::Vulkan,
    window: window::Window,

    pressed_keys: HashSet<VirtualKeyCode>,

    window_size: (u32, u32),
    mouse_locked: bool,

    yaw: f32,
    pitch: f32,

    origin: Vec3A,
    right: Vec3A,
    ahead: Vec3A,
    up: Vec3A,
    // font: pygame.font.Font

    // fps: i32,
    // fps_surface: pygame.Surface
    // fps_data: bytes

    // framebuffer: Framebuffer
    // data: Texture
    // sampler: Sampler
}

impl App {
    fn elapsed_secs(&self) -> f32 {
        SystemTime::now()
            .duration_since(self.program_start)
            .unwrap()
            .as_secs_f32()
    }

    fn reinitialize_images(&mut self) -> error::VResult<()> {
        let vulkan = &mut self.vulkan;
        let image_size = vulkan.surface_info.surface_resolution;

        self.data_image =
            vulkan.new_multi_image("data", vk::Format::R32G32B32A32_SFLOAT, image_size, None)?;
        Ok(())
    }

    fn update_globals(&self) {
        let globals_buffer = self.globals_buffer.mapped(0);
        unsafe {
            let target = globals_buffer.cast::<u32>();
            *target.add(0) = self.quadrants[0].len().try_into().unwrap();
            *target.add(1) = self.quadrants[1].len().try_into().unwrap();
            *target.add(2) = self.quadrants[2].len().try_into().unwrap();
            *target.add(3) = self.quadrants[3].len().try_into().unwrap();
        }
    }

    fn update_view(&self) {
        let view_buffer = self.view_buffer.mapped(0);
        unsafe {
            let target = view_buffer.cast::<u32>();
            *target.add(0) = self.window_size.0;
            *target.add(1) = self.window_size.1;
            let target = view_buffer.cast::<f32>();
            *target.add(2) = FRAC_PI_2;
            let target = view_buffer.cast::<Vec3A>();
            *target.add(1) = self.origin;
            *target.add(2) = self.right;
            *target.add(3) = self.ahead;
            *target.add(4) = self.up;
        }
    }

    fn update_quadrants(&self) {
        unsafe {
            for (quadrant, quadrant_buffer) in
                self.quadrants.iter().zip(self.quadrant_buffers.iter())
            {
                let target = quadrant_buffer.mapped(0);
                for (index, tile_index) in quadrant.iter().enumerate() {
                    let tile_target = target.add(index * TILE_).cast::<i32>();
                    if let Some(tile_index) = tile_index {
                        let tile = &self.tiles[*tile_index];

                        *tile_target = 1;
                        *tile_target.add(1) = 1;

                        for (segment_index, segment) in tile.segments.iter().enumerate() {
                            *tile_target.add(2 + 3 * segment_index + 0) = segment.form as i32;
                            *tile_target.add(2 + 3 * segment_index + 1) = segment.terrain as i32;
                            *tile_target.add(2 + 3 * segment_index + 2) = segment.rotation;
                        }

                        for segment_index in tile.segments.len()..6 {
                            *tile_target.add(2 + 3 * segment_index + 0) = 0;
                        }
                    } else {
                        *tile_target = 0;
                    }
                }
            }
        }
    }

    pub fn new(event_loop: &event_loop::EventLoop) -> error::VResult<Self> {
        // let mut stream = File::open("other/dorfromantik.dump").expect("Failed to open file");
        let mut stream = File::open("other/dorfromantik.dump").expect("Failed to open file");
        let parsed = nrbf_rs::parse_nrbf(&mut stream);
        let savegame = SaveGame::try_from(&parsed).unwrap();
        let tiles = savegame.tiles.iter().map(Tile::from).collect::<Vec<_>>();

        let mut quadrants = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];

        for (index, tile) in tiles.iter().enumerate() {
            let quadrant = &mut quadrants[tile.quadrant()];
            let t_index = tile.index();
            if quadrant.len() < t_index + 1 {
                quadrant.resize(t_index + 1, None);
            }
            quadrant[t_index] = Some(index);
        }

        let shader_paths = vec![
            std::path::Path::new("shaders/data.comp"),
            std::path::Path::new("shaders/lighting.comp"),
        ];
        let window = window::Window::new(event_loop, false)?;
        let window_size = window.size();
        let mut vulkan = vulkan::Vulkan::new(&window, &shader_paths, true)?;

        let image_size = vulkan.surface_info.surface_resolution;
        let data_image =
            vulkan.new_multi_image("data", vk::Format::R32G32B32A32_SFLOAT, image_size, None)?;

        let globals_buffer_size = 1 * IVEC4_;
        let globals_buffer = vulkan.new_multi_buffer(
            "GlobalsBuffer",
            BufferUsage::Uniform,
            globals_buffer_size,
            Some(1),
        )?;

        let view_buffer_size = 2 * INT_ + 2 * FLOAT_ + 4 * VEC3_;
        let view_buffer =
            vulkan.new_multi_buffer("view", BufferUsage::Uniform, view_buffer_size, Some(1))?;

        let mut create_quadrant_buffer = |index: usize, tile_count: usize| {
            let name = format!("QuadrantBuffer{index}");
            let usage = BufferUsage::Storage;
            let size = tile_count * TILE_;
            vulkan.new_multi_buffer(&name, usage, size, Some(1))
        };

        let quadrant_buffers = [
            create_quadrant_buffer(0, quadrants[0].len())?,
            create_quadrant_buffer(1, quadrants[1].len())?,
            create_quadrant_buffer(2, quadrants[2].len())?,
            create_quadrant_buffer(3, quadrants[3].len())?,
        ];

        let mut app = Self {
            // Game logic data.
            savegame,
            tiles,
            quadrants,
            // Shader resources.
            program_start: SystemTime::now(),
            globals_buffer,
            view_buffer,
            quadrant_buffers,
            // images: Vec::new(),
            vulkan,
            window,
            data_image,
            // Window data.
            pressed_keys: Default::default(),
            window_size,
            mouse_locked: false,
            // View.
            yaw: 0.0,
            pitch: 0.0,
            origin: Vec3A::new(0.0, 10.0, 0.0),
            right: Vec3A::new(0.0, 0.0, 1.0),
            ahead: Vec3A::new(1.0, 0.0, 0.0),
            up: Vec3A::new(0.0, 1.0, 0.0),
        };
        app.reinitialize_images()?;
        app.update_globals();
        app.update_quadrants();
        app.update_view();
        Ok(app)
    }

    fn move_mouse(&mut self, x: f32, y: f32) {
        if self.mouse_locked {
            let dx = x - self.window_size.0 as f32 / 2.0;
            let dy = y - self.window_size.1 as f32 / 2.0;

            let sensitivity = 1.0 / 3000.0;

            let yaw = self.yaw - dx * sensitivity;
            let pitch = (-FRAC_PI_2).max((self.pitch - dy * sensitivity).min(FRAC_PI_2));

            self.set_yaw_pitch(yaw, pitch);
            self.window
                .set_cursor_position(self.window_size.0 / 2, self.window_size.1 / 2);
        }
    }

    fn lock_mouse(&mut self, lock: bool) {
        if lock {
            self.window
                .set_cursor_position(self.window_size.0 / 2, self.window_size.1 / 2);
        }
        self.mouse_locked = lock;
        self.window.set_cursor_grab(lock);
    }

    fn set_yaw_pitch(&mut self, yaw: f32, pitch: f32) {
        self.yaw = yaw;
        self.pitch = pitch;

        self.ahead = Vec3A::new(1.0, 0.0, 0.0);
        self.right = Vec3A::new(0.0, 0.0, 1.0);

        let yaw_rotation = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), yaw);
        self.ahead = yaw_rotation * self.ahead;
        self.right = yaw_rotation * self.right;

        let pitch_rotation = Quat::from_axis_angle(self.right.into(), pitch);
        self.ahead = pitch_rotation * self.ahead;

        self.up = self.right.cross(self.ahead);

        self.update_view();
    }

    fn run_vulkan(
        &mut self,
        push_constant_values: std::collections::HashMap<String, vulkan::Value>,
    ) -> error::VResult<()> {
        match unsafe { self.vulkan.tick(&push_constant_values)? } {
            None => (),
            Some(vulkan::Event::Resized) => self.reinitialize_images()?,
        }
        Ok(())
    }
}

fn b_to_f(b: bool) -> f32 {
    if b {
        1.0
    } else {
        0.0
    }
}

impl event_loop::App for App {
    fn tick(&mut self) -> event_loop::ControlFlow {
        use vulkan::Value::F32;

        let mut movement = Vec3A::new(0.0, 0.0, 0.0);

        let w = self.pressed_keys.contains(&VirtualKeyCode::W);
        let s = self.pressed_keys.contains(&VirtualKeyCode::S);
        movement += self.ahead * (b_to_f(w) - b_to_f(s));

        let a = self.pressed_keys.contains(&VirtualKeyCode::A);
        let d = self.pressed_keys.contains(&VirtualKeyCode::D);
        movement += self.right * (b_to_f(d) - b_to_f(a));

        let sqr_len = movement.length_squared();
        if sqr_len > 0.5 {
            let shft = self.pressed_keys.contains(&VirtualKeyCode::LShift);
            let ctrl = self.pressed_keys.contains(&VirtualKeyCode::LControl);

            let mut factor = 0.1 / sqr_len.sqrt();
            if shft {
                factor *= 10.0;
            }
            if ctrl {
                factor *= 0.1;
            }

            self.origin += movement * factor;
            self.origin.y = self.origin.y.max(1.0);
            self.update_view();
        }

        let push_constant_values =
            std::collections::HashMap::from([("time".to_owned(), F32(self.elapsed_secs()))]);

        let result = match self.run_vulkan(push_constant_values) {
            Ok(()) => event_loop::ControlFlow::Continue,
            Err(err) => {
                log::error!("{err}");
                event_loop::ControlFlow::Exit(1)
            }
        };

        // Shouldn't vulkan do this?
        self.vulkan.num_frames += 1;

        result
    }

    fn handle_event(&mut self, event: &Event) -> event_loop::ControlFlow {
        match event {
            Event::Close => ControlFlow::Exit(0),
            Event::Key(ElementState::Pressed, VirtualKeyCode::Escape | VirtualKeyCode::Q) => {
                ControlFlow::Exit(0)
            }
            Event::Key(ElementState::Pressed, key) => {
                self.pressed_keys.insert(key.clone());
                match key {
                    VirtualKeyCode::M => {
                        self.lock_mouse(false);
                    }
                    _ => {}
                };
                ControlFlow::Continue
            }
            Event::Key(ElementState::Released, key) => {
                self.pressed_keys.remove(key);
                ControlFlow::Continue
            }
            Event::MouseMove(x, y) => {
                self.move_mouse(*x, *y);
                ControlFlow::Continue
            }
            Event::MouseButton(ElementState::Pressed, MouseButton::Left) => {
                self.lock_mouse(true);
                ControlFlow::Continue
            }
            _ => ControlFlow::Continue,
        }
    }
}

impl Drop for App {
    fn drop(&mut self) {
        self.vulkan.wait_idle();
    }
}
