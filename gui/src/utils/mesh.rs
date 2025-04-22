use enum_map::{Enum, EnumMap};
use std::sync::{LazyLock, Mutex};
use egui::{epaint::Vertex, Color32, Mesh, Pos2, TextureHandle, TextureId, Vec2};

fn is_point_in_triangle(p: &Pos2, a: &Pos2, b: &Pos2, c: &Pos2) -> bool {
    let v0 = *c - *a;
    let v1 = *b - *a;
    let v2 = *p - *a;

    let dot00 = v0.dot(v0);
    let dot01 = v0.dot(v1);
    let dot02 = v0.dot(v2);
    let dot11 = v1.dot(v1);
    let dot12 = v1.dot(v2);

    let denom = dot00 * dot11 - dot01 * dot01;
    if denom == 0.0 {
        return false; // Avoid division by zero
    }

    let u = (dot11 * dot02 - dot01 * dot12) / denom;
    let v = (dot00 * dot12 - dot01 * dot02) / denom;

    return (u >= 0.0) && (v >= 0.0) && (u + v <= 1.0)
}

pub fn is_point_in_mesh(point: &Pos2, mesh: Mesh) -> bool {
    for triangle in mesh.indices.chunks(3) {
        if triangle.len() < 3 {
            continue;
        }

        let a = mesh.vertices[triangle[0] as usize].pos;
        let b = mesh.vertices[triangle[1] as usize].pos;
        let c = mesh.vertices[triangle[2] as usize].pos;

        if is_point_in_triangle(point, &a, &b, &c) {
            return true;
        }
    }
    
    false
}

pub struct ColoredTexture {
    texture_key: TextureKey,
    color: Color32
}

impl ColoredTexture {
    
    pub fn new(texture_key: TextureKey, color: Color32) -> Self {
        Self {
            texture_key,
            color
        }
    }

}

pub fn create_mesh(positions: &Vec<Pos2>, colored_texture: ColoredTexture) -> Mesh {

    let min = positions.iter().fold(Pos2::new(f32::MAX, f32::MAX), |a, b| a.min(*b));
    let max = positions.iter().fold(Pos2::new(f32::MIN, f32::MIN), |a, b| a.max(*b));
    let size = max - min;

    let vertices = positions.iter().map(|p|  
        Vertex {
            pos: *p,
            uv: (Vec2::new((p.x - min.x) / size.x, (p.y - min.y) / size.y) * size / 32.0).to_pos2(),
            color: colored_texture.color
        }
    ).collect::<Vec<_>>();

    let indices = earcutr::earcut(
        &positions.iter().map(|p| vec![p.x, p.y]).flatten().collect::<Vec<_>>(),
        &[],
        2,
    ).unwrap_or_default().iter().map(|i| *i as u32).collect::<Vec<_>>();

    return Mesh {
        indices: indices,
        vertices: vertices,
        texture_id: get_texture_id(colored_texture.texture_key)
    };

}


//All textures loaded using this macro are kept in GPU memory for the whole lifetime of the program, so use this macro responsibly!
macro_rules! define_textures {
    (
        $(
            $name:ident => $path:expr
        ),* $(,)?
    ) => {

        #[derive(Enum, Clone)]
        pub enum TextureKey {
            $( $name ),*
        }

        pub fn register_textures(ctx: &egui::Context) {
            const ALL_TEXTURE_KEYS: &[TextureKey] = &[
                $( TextureKey::$name ),*
            ];

            for key in ALL_TEXTURE_KEYS {
                let image = image::load_from_memory(get_texture_data(key)).unwrap().to_rgba8();
                let (width, height) = image.dimensions();
                let pixels: Vec<Color32> = image
                    .pixels()
                    .map(|p| Color32::from_rgba_premultiplied(p[0], p[1], p[2], p[3]))
                    .collect();
                
                let texture = egui::ColorImage {
                    size: [width as usize, height as usize],
                    pixels,
                };
        
                TEXTURE_IDS.lock().unwrap()[key.clone()] = Some(ctx.load_texture(stringify!(key), texture, egui::TextureOptions::LINEAR_REPEAT));
            }
        }

        //Panics if register_textures is not called first
        pub fn get_texture_id(key: TextureKey) -> TextureId {
            return TEXTURE_IDS.lock().unwrap()[key].as_ref().unwrap().id();
        }

        static TEXTURE_IDS: LazyLock<Mutex<EnumMap<TextureKey, Option<TextureHandle>>>> = LazyLock::new(|| Mutex::new(EnumMap::default()));

        fn get_texture_data(key: &TextureKey) -> &'static [u8] {
            match key {
                $( TextureKey::$name => include_bytes!($path).as_slice()),*
            }
        }
    }
}

define_textures! {
    PatternFull => "./../../assets/textures/full_pattern_32x32.png",
    PatternDiagonal => "./../../assets/textures/diagonal_pattern_32x32.png",
    PatternCrosshatch => "./../../assets/textures/crosshatch_pattern_32x32.png",
    PatternDots => "./../../assets/textures/dots_pattern_32x32.png",
}