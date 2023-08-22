//! Contains code for a map widget. TODO: replace with a proper map.

use std::collections::{HashMap, HashSet};
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;
use std::io::{Read, Write};
use std::f64::consts::TAU;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use instant::Instant;

use slippy_map_tiles::{BBox, Tile};

use eframe::egui;
use egui::widgets::plot::{Line, PlotBounds, PlotImage, PlotPoint};
use egui::{Context, Color32, ColorImage, TextureHandle, Vec2};
use egui::mutex::Mutex;

use crate::state::*;

fn tile_mapbox_url(tile: &Tile, access_token: &String) -> String {
    format!("https://api.mapbox.com/styles/v1/mapbox/satellite-streets-v12/tiles/512/{}/{}/{}@2x?access_token={}",
        tile.zoom(),
        tile.x(),
        tile.y(),
        access_token
    )
}

fn tile_id(tile: &Tile) -> String {
    format!("{:02x}{:08x}{:08x}", tile.zoom(), tile.x(), tile.y())
}

#[cfg(not(target_arch = "wasm32"))]
fn load_tile_bytes(tile: &Tile, access_token: &String) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    let project_dirs = directories::ProjectDirs::from("space", "tudsat", "sam").unwrap();
    let cache_dir = project_dirs.cache_dir();
    if !cache_dir.exists() {
        std::fs::create_dir_all(cache_dir)?;
    }

    let path = cache_dir.join(format!("{}.png", tile_id(tile)));

    if path.exists() {
        let mut f = std::fs::File::open(path)?;
        let mut buffer = Vec::new();
        f.read_to_end(&mut buffer)?;
        return Ok(buffer);
    }

    let response = reqwest::blocking::get(tile_mapbox_url(&tile, access_token))?
        .error_for_status()?;
    let bytes = response.bytes()?.to_vec();

    let mut f = std::fs::File::create(path)?;
    f.write_all(&bytes)?;

    Ok(bytes)
}

#[cfg(not(target_arch = "wasm32"))]
fn load_tile_image(tile: &Tile, access_token: &String) -> Result<ColorImage, Box<dyn std::error::Error>> {
    let bytes = load_tile_bytes(tile, access_token)?;
    let image = egui_extras::image::load_image_bytes(&bytes)?;
    Ok(image)
}

#[cfg(target_arch = "wasm32")]
async fn load_tile_bytes(tile: &Tile, access_token: &String) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    let url = tile_mapbox_url(&tile, access_token);
    let response = reqwest::get(url).await?
        .error_for_status()?;
    let bytes = response.bytes().await?.to_vec();

    Ok(bytes)
}

#[cfg(target_arch = "wasm32")]
async fn load_tile_image(tile: &Tile, access_token: &String) -> Result<ColorImage, Box<dyn std::error::Error>> {
    let bytes = load_tile_bytes(tile, access_token).await?;
    let image = egui_extras::image::load_image_bytes(&bytes)?;
    Ok(image)
}

/// Cached data for map, including satellite imagery tiles, which are also
/// cached on disk.
pub struct TileCache {
    textures: HashMap<String, TextureHandle>,
    tiles: HashMap<String, ColorImage>,
    loading: HashSet<String>,
}

impl TileCache {
    pub fn new() -> Self {
        Self {
            textures: HashMap::new(),
            tiles: HashMap::new(),
            loading: HashSet::new(),
        }
    }

    pub fn cached_image(&mut self, ctx: &Context, tile: &Tile) -> Option<PlotImage> {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        let center = tile.center_point();
        let center = PlotPoint::new(center.lon(), center.lat());
        let size = Vec2::new(tile.right() - tile.left(), tile.top() - tile.bottom());

        let tile_id = tile_id(tile);
        let texture_id = format!("map_tile_{}", &tile_id);

        if let Some(texture) = self.textures.get(&tile_id) {
            return Some(PlotImage::new(texture, center, size));
        }

        if let Some(ci) = self.tiles.get(&tile_id) {
            let texture = ctx.load_texture(&texture_id, ci.clone(), Default::default());
            self.textures.insert(tile_id, texture.clone());
            return Some(PlotImage::new(&texture, center, size));
        }

        None
    }

    pub fn insert(&mut self, tile: Tile, image: ColorImage) {
        self.tiles.insert(tile_id(&tile), image);
    }
}

pub struct MapCache {
    points: Vec<(f64, f64, f64)>,
    plot_points: Vec<(Vec<[f64; 2]>, Color32)>,
    max_alt: f64,
    pub center: (f64, f64),
    pub hdop_circle_points: Option<Vec<[f64; 2]>>,
    gradient_lookup: Vec<Color32>,
}

impl MapCache {
    pub fn new() -> Self {
        let gradient_lookup = (0..=100)
            .map(|i| colorgrad::yl_or_rd().at((i as f64) / 100.0).to_rgba8())
            .map(|color| Color32::from_rgb(color[0], color[1], color[2]))
            .collect();

        MapCache {
            points: Vec::new(),
            plot_points: Vec::new(),
            max_alt: 300.0,
            center: (49.861445, 8.68519),
            hdop_circle_points: None,
            gradient_lookup
        }
    }

    pub fn push(&mut self, _x: Instant, vs: &VehicleState) {
        if let (Some(lat), Some(lng)) = (vs.latitude, vs.longitude) {
            let (lat, lng) = (lat as f64, lng as f64);
            let altitude_ground = vs.altitude_ground.unwrap_or(0.0);
            let alt = vs.altitude_gps.map(|alt| alt - altitude_ground).unwrap_or(0.0) as f64;
            let hdop = vs.hdop.unwrap_or(9999);

            self.points.push((lat, lng, alt));
            self.max_alt = f64::max(self.max_alt, alt);
            self.center = (lat, lng);

            self.plot_points = self.points.windows(2)
                .map(|pair| {
                    #[cfg(feature = "profiling")]
                    puffin::profile_scope!("map_line_creation");

                    let points = vec![[pair[0].1, pair[0].0], [pair[1].1, pair[1].0]];
                    let color = self.gradient_lookup[((1.0 - pair[0].2 / self.max_alt) * 100.0).floor() as usize];
                    (points, color)
                    })
                .collect();

            let cep_m = 2.5 * (hdop as f64) / 100.0;
            let r = 360.0 * cep_m / 40_075_017.0; // meters to decimal degrees
            let points = (0..=64)
                .map(|i| (i as f64) * TAU / 64.0)
                .map(|i| [r * i.cos() * self.aspect() + lng, r * i.sin() + lat])
                .collect();
            self.hdop_circle_points = Some(points);
        }
    }

    pub fn reset(&mut self) {
        self.points.truncate(0);
        self.plot_points.truncate(0);
        self.max_alt = 300.0;
        self.center = (49.861445, 8.68519);
        self.hdop_circle_points = None;
    }

    pub fn aspect(&self) -> f64 {
        1.0 / f64::cos(self.center.0.to_radians() as f64)
    }

    fn lines<'a>(&'a self) -> Vec<Line> {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        self.plot_points.iter()
            .map(|(pp, color)| {
                Line::new(pp.clone()).width(2.0).color(color.clone())
            })
            .collect()
    }

    pub fn hdop_circle_line(&self) -> Option<Line> {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        self.hdop_circle_points.as_ref()
            .map(|points| Line::new(points.clone()).width(1.5).color(Color32::RED))
    }
}

/// State of the map widget, stored by the application.
#[derive(Clone)]
pub struct MapState {
    pub tile_cache: Arc<Mutex<TileCache>>,
    pub cache: Rc<RefCell<MapCache>>,
    access_token: String,
}

impl MapState {
    pub fn new(access_token: String) -> Self {
        Self {
            tile_cache: Arc::new(Mutex::new(TileCache::new())),
            cache: Rc::new(RefCell::new(MapCache::new())),
            access_token
        }
    }

    pub fn set_access_token(&mut self, token: String) {
        self.access_token = token;
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn load_tile(&self, tile: Tile) {
        let cache = self.tile_cache.clone();
        let at = self.access_token.clone();
        std::thread::spawn(move || {
            match load_tile_image(&tile, &at) {
                Ok(image) => cache.lock().insert(tile, image),
                Err(e) => log::error!("{:?}", e),
            }

            cache.lock().loading.remove(&tile_id(&tile));
        });
    }

    #[cfg(target_arch = "wasm32")]
    fn load_tile(&self, tile: Tile) {
        let cache = self.tile_cache.clone();
        wasm_bindgen_futures::spawn_local(async move {
            match load_tile_image(&tile, &self.access_token).await {
                Ok(image) => cache.lock().insert(tile, image),
                Err(e) => log::error!("{:?}", e),
            }

            cache.lock().loading.remove(&tile_id(&tile));
        });
    }

    pub fn tile_images<'a>(&'a self, ctx: &'a Context, bounds: PlotBounds) -> Box<dyn Iterator<Item = PlotImage> + 'a> {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        let width = bounds.max()[0] - bounds.min()[0];
        let height = bounds.max()[1] - bounds.min()[1];

        const NUM_TILES: f64 = 4.0;
        let world_prop = f64::max(width / 180.0, height / 180.0);
        let zoom = f64::round(f64::log2(world_prop / NUM_TILES)).abs();
        let zoom = f64::max(4.0, f64::min(19.0, zoom));

        let bbox = BBox::new(
            bounds.max()[1] as f32,
            bounds.min()[0] as f32,
            bounds.min()[1] as f32,
            bounds.max()[0] as f32
        ).unwrap_or(BBox::new(89.0, -179.0, -89.0, 179.0).unwrap());

        let iter = bbox.tiles_for_zoom(zoom as u8)
            .filter_map(|tile| {
                let result = self.tile_cache.lock().cached_image(ctx, &tile);
                if result.is_none() && self.tile_cache.lock().loading.insert(tile_id(&tile)) {
                    self.load_tile(tile);
                }
                result
            });
        Box::new(iter)
    }

    pub fn push(&self, x: Instant, vs: &VehicleState) {
        self.cache.borrow_mut().push(x, vs);
    }

    pub fn reset(&self) {
        self.cache.borrow_mut().reset();
    }
}

pub trait MapUiExt {
    fn map(
        &mut self,
        state: &MapState
    );
}

impl MapUiExt for egui::Ui {
    fn map(
        &mut self,
        state: &MapState
    ) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        let cache = state.cache.borrow_mut();

        self.vertical_centered(|ui| {
            let plot = egui::widgets::plot::Plot::new("map")
                .allow_scroll(false)
                .data_aspect(cache.aspect() as f32)
                .set_margin_fraction(egui::Vec2::new(0.0, 0.15))
                .show_axes([false, false])
                .include_x(cache.center.1 - 0.005)
                .include_x(cache.center.1 + 0.005)
                .include_y(cache.center.0 - 0.005)
                .include_y(cache.center.0 + 0.005);

            plot.show(ui, |plot_ui| {
                let ctx = plot_ui.ctx().clone();
                for pi in state.tile_images(&ctx, plot_ui.plot_bounds()) {
                    #[cfg(feature = "profiling")]
                    puffin::profile_scope!("tile_image");

                    plot_ui.image(pi);
                }

                if let Some(line) = cache.hdop_circle_line() {
                    #[cfg(feature = "profiling")]
                    puffin::profile_scope!("hdop_circle");

                    plot_ui.line(line);
                }

                for line in cache.lines() {
                    #[cfg(feature = "profiling")]
                    puffin::profile_scope!("map_line");

                    plot_ui.line(line);
                }
            });
        });
    }
}
