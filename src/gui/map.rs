//! Contains code for a map widget. TODO: replace with a proper map.

use std::collections::{HashMap, HashSet};
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;
use std::io::{Read, Write};
use std::f64::consts::TAU;

use slippy_map_tiles::{BBox, Tile};

use eframe::egui;
use egui::widgets::plot::{Line, PlotBounds, PlotImage, PlotPoint};
use egui::{Context, Color32, ColorImage, TextureHandle, Vec2};
use egui::mutex::Mutex;

use crate::state::*;

const MAPBOX_ACCESS_TOKEN: &str = "pk.eyJ1Ijoia29mZmVpbmZsdW1taSIsImEiOiJjbGE0cDl4MWkwcXJoM3VxcXBmeHJhdGpzIn0.Md170HfUJM_BLss3zb0bMg";

fn tile_mapbox_url(tile: &Tile) -> String {
    format!("https://api.mapbox.com/styles/v1/mapbox/satellite-streets-v12/tiles/512/{}/{}/{}@2x?access_token={}",
        tile.zoom(),
        tile.x(),
        tile.y(),
        MAPBOX_ACCESS_TOKEN
    )
}

fn tile_id(tile: &Tile) -> String {
    format!("{:02x}{:08x}{:08x}", tile.zoom(), tile.x(), tile.y())
}

#[cfg(not(target_arch = "wasm32"))]
fn load_tile_bytes(tile: &Tile) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    // TODO: non-linux cache part
    let name = format!("{}.png", tile_id(tile));
    let base_dirs = xdg::BaseDirectories::with_prefix("sam")?;
    let path = base_dirs.get_cache_file(name);

    if path.exists() {
        let mut f = std::fs::File::open(path)?;
        let mut buffer = Vec::new();
        f.read_to_end(&mut buffer)?;
        return Ok(buffer);
    }

    let response = reqwest::blocking::get(tile_mapbox_url(&tile))?
        .error_for_status()?;
    let bytes = response.bytes()?.to_vec();

    let mut f = std::fs::File::create(path)?;
    f.write_all(&bytes)?;

    Ok(bytes)
}

#[cfg(not(target_arch = "wasm32"))]
fn load_tile_image(tile: &Tile) -> Result<ColorImage, Box<dyn std::error::Error>> {
    let bytes = load_tile_bytes(tile)?;
    let image = egui_extras::image::load_image_bytes(&bytes)?;
    Ok(image)
}

#[cfg(target_arch = "wasm32")]
async fn load_tile_bytes(tile: &Tile) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    let url = tile_mapbox_url(&tile);
    let response = reqwest::get(url).await?
        .error_for_status()?;
    let bytes = response.bytes().await?.to_vec();

    Ok(bytes)
}

#[cfg(target_arch = "wasm32")]
async fn load_tile_image(tile: &Tile) -> Result<ColorImage, Box<dyn std::error::Error>> {
    let bytes = load_tile_bytes(tile).await?;
    let image = egui_extras::image::load_image_bytes(&bytes)?;
    Ok(image)
}

/// Cached data for map, including satellite imagery tiles, which are also
/// cached on disk.
pub struct MapCache {
    textures: HashMap<String, TextureHandle>,
    tiles: HashMap<String, ColorImage>,
    loading: HashSet<String>,
}

impl MapCache {
    pub fn new() -> Self {
        Self {
            textures: HashMap::new(),
            tiles: HashMap::new(),
            loading: HashSet::new(),
        }
    }

    pub fn cached_image(&mut self, ctx: &Context, tile: &Tile) -> Option<PlotImage> {
        let center = tile.center_point();
        let center = PlotPoint::new(center.lon(), center.lat());
        let size = Vec2::new(tile.right() - tile.left(), tile.top() - tile.bottom());

        let id = tile_id(&tile);
        if let Some(texture) = self.textures.get(&id) {
            return Some(PlotImage::new(texture, center, size));
        }

        if let Some(ci) = self.tiles.get(&tile_id(tile)) {
            let id = format!("map_tile_{}", &id);
            let texture = ctx.load_texture(&id, ci.clone(), Default::default());
            self.textures.insert(id, texture.clone());
            return Some(PlotImage::new(&texture, center, size));
        }

        None
    }

    pub fn insert(&mut self, tile: Tile, image: ColorImage) {
        self.tiles.insert(tile_id(&tile), image);
    }
}

/// State of the map widget, stored by the application.
#[derive(Clone)]
pub struct MapState {
    pub vehicle_states: Rc<RefCell<Vec<VehicleState>>>,
    pub cache: Arc<Mutex<MapCache>>,
}

impl MapState {
    pub fn new(vehicle_states: Rc<RefCell<Vec<VehicleState>>>) -> Self {
        Self {
            vehicle_states,
            cache: Arc::new(Mutex::new(MapCache::new())),
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn load_tile(&self, tile: Tile) {
        let cache = self.cache.clone();
        std::thread::spawn(move || {
            match load_tile_image(&tile) {
                Ok(image) => cache.lock().insert(tile, image),
                Err(e) => log::error!("{:?}", e),
            }

            cache.lock().loading.remove(&tile_id(&tile));
        });
    }

    #[cfg(target_arch = "wasm32")]
    fn load_tile(&self, tile: Tile) {
        let cache = self.cache.clone();
        wasm_bindgen_futures::spawn_local(async move {
            match load_tile_image(&tile).await {
                Ok(image) => cache.lock().insert(tile, image),
                Err(e) => log::error!("{:?}", e),
            }

            cache.lock().loading.remove(&tile_id(&tile));
        });
    }

    pub fn tile_images<'a>(&'a self, ctx: &'a Context, bounds: PlotBounds) -> Box<dyn Iterator<Item = PlotImage> + 'a> {
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
                let result = self.cache.lock().cached_image(ctx, &tile);
                if result.is_none() && self.cache.lock().loading.insert(tile_id(&tile)) {
                    self.load_tile(tile);
                }
                result
            });
        Box::new(iter)
    }
}

pub trait MapUiExt {
    fn map(
        &mut self,
        state: MapState
    );
}

impl MapUiExt for egui::Ui {
    fn map(
        &mut self,
        state: MapState
    ) {
        let points: Vec<(f64, f64, f64, u16)> = state.vehicle_states.borrow()
            .iter()
            .filter(|vs| vs.latitude.is_some() && vs.longitude.is_some())
            .map(|vs| {
                let lat = vs.latitude.unwrap();
                let lng = vs.longitude.unwrap();
                let altitude_ground = vs.altitude_ground.unwrap_or(0.0);
                let alt = vs.altitude_gps.map(|alt| alt - altitude_ground).unwrap_or(0.0);
                (lat as f64, lng as f64, alt as f64, vs.hdop.unwrap_or(9999))
            })
            .collect();

        let max_alt = points.iter()
            .map(|(_, _, alt, _)| alt)
            .fold(300.0, |a, b| f64::max(a, *b));
        let lines = points.windows(2)
            .map(|pair| {
                let points = vec![[pair[0].1, pair[0].0], [pair[1].1, pair[1].0]];
                let color = colorgrad::yl_or_rd().at(1.0 - pair[0].2 / max_alt).to_rgba8();
                Line::new(points)
                    .width(2.0)
                    .color(Color32::from_rgb(color[0], color[1], color[2]))
            });

        self.vertical_centered(|ui| {
            let last = points.last();
            let center = last.cloned().unwrap_or((49.861445, 8.68519, 0.0, 0));
            let aspect = 1.0 / f64::cos(center.0.to_radians() as f64);
            let plot = egui::widgets::plot::Plot::new("map")
                .allow_scroll(false)
                .data_aspect(aspect as f32)
                .set_margin_fraction(egui::Vec2::new(0.0, 0.15))
                .show_axes([false, false])
                .include_x(center.1 - 0.005)
                .include_x(center.1 + 0.005)
                .include_y(center.0 - 0.005)
                .include_y(center.0 + 0.005);

            plot.show(ui, |plot_ui| {
                let ctx = plot_ui.ctx().clone();
                for pi in state.tile_images(&ctx, plot_ui.plot_bounds()) {
                    plot_ui.image(pi);
                }

                // TODO: maybe cache this?
                if let Some((lat, lng, _, hdop)) = last {
                    let cep_m = 2.5 * (*hdop as f64) / 100.0;
                    let r = 360.0 * cep_m / 40_075_017.0; // meters to decimal degrees
                    let circle_points: egui::widgets::plot::PlotPoints = (0..=64)
                        .map(|i| (i as f64) * TAU / 64.0)
                        .map(|i| [r * i.cos() * aspect + lng, r * i.sin() + lat])
                        .collect();
                    plot_ui.line(Line::new(circle_points).width(1.5).color(Color32::RED));
                }

                for line in lines {
                    plot_ui.line(line);
                }
            });
        });
    }
}
