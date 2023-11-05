//! Contains code for a map widget. TODO: replace with a proper map.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::f64::consts::TAU;
use std::io::{Read, Write};
use std::rc::Rc;
use std::sync::Arc;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use slippy_map_tiles::{BBox, Tile};

use eframe::egui;
use egui::mutex::Mutex;
use egui_plot::{Line, PlotBounds, PlotImage, PlotPoint, PlotPoints};
use egui::{Color32, ColorImage, Context, TextureHandle, Vec2};

use crate::data_source::DataSource;

const GRADIENT_MAX_ALT: f64 = 10000.0;

fn tile_mapbox_url(tile: &Tile, access_token: &String) -> String {
    format!(
        "https://api.mapbox.com/styles/v1/mapbox/satellite-streets-v12/tiles/512/{}/{}/{}@2x?access_token={}",
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
    #[cfg(not(target_os="android"))]
    let project_dirs = directories::ProjectDirs::from("space", "tudsat", "sam").unwrap();
    #[cfg(not(target_os="android"))]
    let cache_dir = project_dirs.cache_dir();

    // TODO: avoid hardcoding this
    #[cfg(target_os="android")]
    let cache_dir = std::path::Path::new("/data/user/0/space.tudsat.sam/cache");

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

    // TODO: do this nicer
    let rt = tokio::runtime::Builder::new_current_thread().enable_io().build().unwrap();
    let result = rt.block_on(reqwest::get(tile_mapbox_url(&tile, access_token)));
    let response = result?.error_for_status()?;
    let bytes = rt.block_on(response.bytes())?.to_vec();

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
    let response = reqwest::get(url).await?.error_for_status()?;
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
    plot_points: Vec<([[f64; 2]; 2], Color32)>,
    max_alt: f64,
    pub center: (f64, f64),
    pub hdop_circle_points: Option<Vec<[f64; 2]>>,
    cached_state: Option<(Instant, usize)>,
    gradient_lookup: Vec<Color32>,
}

impl MapCache {
    pub fn new() -> Self {
        let gradient_lookup = (0..=1000)
            .map(|i| colorgrad::sinebow().at((i as f64) / 1000.0).to_rgba8())
            .map(|color| Color32::from_rgb(color[0], color[1], color[2]))
            .collect();

        MapCache {
            points: Vec::new(),
            plot_points: Vec::new(),
            max_alt: 300.0,
            center: (49.861445, 8.68519),
            hdop_circle_points: None,
            cached_state: None,
            gradient_lookup,
        }
    }

    pub fn aspect(&self) -> f64 {
        1.0 / f64::cos(self.center.0.to_radians() as f64)
    }

    fn update_position_cache(&mut self, data_source: &dyn DataSource, keep_first: usize) {
        let new_data = data_source.vehicle_states()
            .skip(keep_first)
            .filter(|(_t, vs)| vs.latitude.is_some() && vs.longitude.is_some())
            .map(|(_t, vs)| (vs.latitude.unwrap(), vs.longitude.unwrap(), vs.altitude_gps.unwrap_or(0.0)))
            .map(|(lat, lng, alt)| (lat as f64, lng as f64, alt as f64));

        if keep_first > 0 {
            self.points.extend(new_data);
        } else {
            self.points = new_data.collect();
        }

        self.plot_points = self
            .points
            .windows(2)
            .map(|pair| {
                let points = [[pair[0].1, pair[0].0], [pair[1].1, pair[1].0]];
                let index = usize::min(((pair[0].2 / GRADIENT_MAX_ALT) * self.gradient_lookup.len() as f64).floor() as usize, self.gradient_lookup.len() - 1);
                let color = self.gradient_lookup[index];
                (points, color)
            })
            .collect();
    }

    fn update_hdop_cache(&mut self, data_source: &dyn DataSource) {
        let last = data_source.vehicle_states().rev().find(|(_, vs)| vs.latitude.is_some() && vs.longitude.is_some());
        if let Some((_, vs)) = last {
            let (lat, lng) = (vs.latitude.unwrap() as f64, vs.longitude.unwrap() as f64);
            let hdop = vs.hdop.unwrap_or(9999);

            let cep_m = 2.5 * (hdop as f64) / 100.0;
            let r = 360.0 * cep_m / 40_075_017.0; // meters to decimal degrees
            let points = (0..=64)
                .map(|i| (i as f64) * TAU / 64.0)
                .map(|i| [r * i.cos() * self.aspect() + lng, r * i.sin() + lat])
                .collect();
            self.center = (lat, lng);
            self.hdop_circle_points = Some(points);
        } else {
            self.hdop_circle_points = None;
        }
    }

    fn update_cache_if_necessary(&mut self, data_source: &dyn DataSource) {
        let new_len = data_source.vehicle_states().len();
        if new_len == 0 {
            self.points.truncate(0);
            self.plot_points.truncate(0);
            self.hdop_circle_points = None;
            self.cached_state = None;
            self.max_alt = 300.0;
            return;
        }

        let (last_t, _) = data_source.vehicle_states().rev().next().unwrap().clone();
        let cached_state = Some((last_t, new_len));

        // We have already cached this exact set of vehicle states, do nothing.
        if cached_state == self.cached_state {
            return;
        }

        // Try to determine if the new data is simply a few more points appended to the previously
        // plotted data, which we have cached. If so, we keep the old and simply append the new
        // points. If not, we recalculate the cache completely.
        let old_len = self.cached_state.map(|(_, l)| l).unwrap_or(0);
        let mut keep_first = (new_len > old_len).then_some(old_len).unwrap_or(0);
        if keep_first > 0 {
            // double-check that it is actually the same set of states by looking for our previous
            // last state in the new data
            let (previous_last, _) = data_source.vehicle_states()
                .rev()
                .skip(new_len - keep_first)
                .next()
                .unwrap();
            if self.cached_state.map(|(t, _)| t != *previous_last).unwrap_or(true) {
                keep_first = 0;
            }
        }

        self.update_position_cache(data_source, keep_first);
        self.update_hdop_cache(data_source);
        self.cached_state = cached_state;
    }

    fn lines<'a>(&'a mut self, data_source: &dyn DataSource) -> Vec<Line> {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        self.update_cache_if_necessary(data_source);
        self.plot_points
            .clone()
            .into_iter()
            .map(|(pp, color)| Line::new(PlotPoints::from_iter(pp.into_iter())).width(2.0).color(color.clone()))
            .collect()
    }

    pub fn hdop_circle_line(&mut self, data_source: &dyn DataSource) -> Option<Line> {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        self.update_cache_if_necessary(data_source);
        self.hdop_circle_points
            .as_ref()
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
            access_token,
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
        let access_token = self.access_token.clone();
        wasm_bindgen_futures::spawn_local(async move {
            match load_tile_image(&tile, &access_token).await {
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

        let bbox =
            BBox::new(bounds.max()[1] as f32, bounds.min()[0] as f32, bounds.min()[1] as f32, bounds.max()[0] as f32)
                .unwrap_or(BBox::new(89.0, -179.0, -89.0, 179.0).unwrap());

        let iter = bbox.tiles_for_zoom(zoom as u8).filter_map(|tile| {
            let result = self.tile_cache.lock().cached_image(ctx, &tile);
            if result.is_none() && self.tile_cache.lock().loading.insert(tile_id(&tile)) {
                self.load_tile(tile);
            }
            result
        });
        Box::new(iter)
    }
}

pub trait MapUiExt {
    fn map(&mut self, state: &MapState, data_source: &dyn DataSource);
}

impl MapUiExt for egui::Ui {
    fn map(&mut self, state: &MapState, data_source: &dyn DataSource) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        let mut cache = state.cache.borrow_mut();

        self.vertical_centered(|ui| {
            let plot = egui_plot::Plot::new("map")
                .allow_scroll(false)
                .data_aspect(cache.aspect() as f32)
                .set_margin_fraction(egui::Vec2::new(0.0, 0.15))
                .show_axes([false, false])
                .show_grid([false, false])
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

                if let Some(line) = cache.hdop_circle_line(data_source) {
                    #[cfg(feature = "profiling")]
                    puffin::profile_scope!("hdop_circle");

                    plot_ui.line(line);
                }

                for line in cache.lines(data_source) {
                    #[cfg(feature = "profiling")]
                    puffin::profile_scope!("map_line");

                    plot_ui.line(line);
                }
            });
        });
    }
}
