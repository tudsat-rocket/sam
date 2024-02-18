//! Contains our map widget, based on the walkers crate.

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use directories::ProjectDirs;
use eframe::egui;
use egui::{Color32, Vec2, Widget, Frame, Rect, Stroke, Layout};
//use walkers::extras::{Places, Place, Style, Lines, Line};
use walkers::extras::{Places, Place, Style};
use walkers::{Tiles, MapMemory, Position, Plugin, HttpOptions};

use crate::data_source::DataSource;

const GRADIENT_MAX_ALT: f64 = 10000.0;

pub struct CrosshairPlugin {
    render_crosshair: bool,

}

impl Plugin for CrosshairPlugin {
    fn run(&mut self, _response: &egui::Response, painter: egui::Painter, _projector: &walkers::Projector) {
        let rect = painter.clip_rect();
        let crosshair_stroke = Stroke {
            width: 1.0,
            color: Color32::GRAY.gamma_multiply(0.8)
        };

        if self.render_crosshair {
            painter.line_segment([rect.center_top(), rect.center_bottom()], crosshair_stroke);
            painter.line_segment([rect.left_center(), rect.right_center()], crosshair_stroke);
        }
    }
}

pub enum Line {
    //Line(Position, Position, egui::Stroke),
    //Path(Vec<Position>, egui::Stroke),
    PathWithValue(Vec<(Position, f64)>, Box<dyn Fn(f64) -> egui::Stroke>),
}

pub struct LinePlugin {
    lines: Vec<Line>
}

impl LinePlugin {
    pub fn new(lines: Vec<Line>) -> Self {
        Self { lines }
    }
}

impl Plugin for LinePlugin {
    fn run(&mut self, _response: &egui::Response, painter: egui::Painter, projector: &walkers::Projector) {
        for l in &self.lines {
            match l {
                //Line::Line(p1, p2, stroke) => {
                //    let p1 = projector.project(*p1).to_pos2();
                //    let p2 = projector.project(*p2).to_pos2();
                //    painter.line_segment([p1, p2], *stroke);
                //},
                //Line::Path(positions, stroke) => {
                //    let screen_positions: Vec<_> = positions.into_iter()
                //        .map(|p| projector.project(*p).to_pos2())
                //        .collect();
                //    for segment in screen_positions.windows(2) {
                //        painter.line_segment([segment[0], segment[1]], *stroke);
                //    }
                //}
                Line::PathWithValue(positions_and_values, f) => {
                    let screen_positions: Vec<_> = positions_and_values.into_iter()
                        .map(|(p, val)| (projector.project(*p).to_pos2(), val))
                        .collect();
                    for segment in screen_positions.windows(2) {
                        painter.line_segment([segment[0].0, segment[1].0], f((segment[0].1 + segment[1].1) / 2.0));
                    }
                }
            }
        }
    }
}

/// Permanent Map data store, kept in memory the entire time
pub struct MapState {
    osm_tiles: Tiles,
    mapbox_tiles: Option<Tiles>,
    memory: MapMemory,
    satellite: bool,
    gradient_lookup: Vec<Color32>,
    position_cache: Vec<(Position, f64)>,
    last_position: Option<(Position, f64, f32)>,
    cached_state: Option<(Instant, usize)>,
}

impl MapState {
    fn http_options() -> HttpOptions {
        // We don't cache anything on web assembly
        #[cfg(target_arch = "wasm32")]
        let cache_path = None;

        // On Android, we just hardcode the path for now. If we wanted to do it properly, we'd
        // have to request a path and pass it to our code via the JNI.
        #[cfg(target_os = "android")]
        let cache_path = Some(std::path::PathBuf::new("/data/user/0/space.tudsat.sam/cache"));

        // On other platforms, we store map tiles on-disk
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        let cache_path = Some(ProjectDirs::from("space", "tudsat", "sam").unwrap().cache_dir().into());

        HttpOptions {
            cache: cache_path,
        }
    }

    pub fn new(ctx: &egui::Context, mapbox_access_token: Option<String>) -> Self {
        let osm_tiles = Tiles::with_options(
            walkers::sources::OpenStreetMap,
            Self::http_options(),
            ctx.to_owned()
        );

        // We only show the mapbox map if we have an access token
        let mapbox_access_token = mapbox_access_token.or(option_env!("MAPBOX_ACCESS_TOKEN").map(|s| s.to_string()));
        let mapbox_tiles = mapbox_access_token.map(|t| Tiles::with_options(
            walkers::sources::Mapbox {
                style: walkers::sources::MapboxStyle::Satellite,
                access_token: t.to_string(),
                high_resolution: true,
            },
            Self::http_options(),
            ctx.to_owned()//,
            //cache
        ));

        // We default to satellite view if we have one.
        let satellite = mapbox_tiles.is_some();

        let gradient_lookup = (0..=1000)
            .map(|i| colorgrad::sinebow().at((i as f64) / 1000.0).to_rgba8())
            .map(|color| Color32::from_rgb(color[0], color[1], color[2]))
            .collect();

        Self {
            osm_tiles,
            mapbox_tiles,
            memory: MapMemory::default(),
            satellite,
            gradient_lookup,
            position_cache: Vec::new(),
            last_position: None,
            cached_state: None,
        }
    }

    pub fn vehicle_positions(&mut self, data_source: &mut dyn DataSource) -> Vec<(Position, f64)> {
        let last = data_source.vehicle_states().rev().next();
        if last.is_none() {
            return Vec::new();
        }
        let state = (last.unwrap().0, data_source.vehicle_states().len());

        // repopulate cache
        if self.cached_state.map(|s| s != state).unwrap_or(true) {
            //let last_position = (

            let all_positions = data_source.vehicle_states()
                .scan((None, None), |(ground_asl, altitude_asl), (_, vs)| {
                    *ground_asl = vs.altitude_ground_asl.or(*ground_asl);
                    *altitude_asl = vs.altitude_asl.or(*altitude_asl);
                    Some((ground_asl.clone(), altitude_asl.clone(), vs))
                })
                .filter(|(_, _, vs)|
                    vs.latitude.is_some() &&
                    vs.longitude.is_some() &&
                    vs.num_satellites.map(|s| s >= 6).unwrap_or(false) &&
                    vs.hdop.map(|h| h < 500).unwrap_or(false)
                )
                .map(|(ground_asl, alt_asl, vs)| {
                    let pos = Position::from_lat_lon(vs.latitude.unwrap() as f64, vs.longitude.unwrap() as f64);
                    let alt = (alt_asl.unwrap_or_default() - ground_asl.unwrap_or_default()) as f64;
                    let hdop = vs.hdop.unwrap_or_default() as f32 / 100.0;
                    self.last_position = Some((pos, alt, hdop));
                    (pos, alt)
                });

            self.position_cache.truncate(0);
            for (pos, alt) in all_positions {
                let add = self.position_cache.last().map(|(last_pos, last_alt)| {
                    (last_alt - alt).abs() > 20.0 ||
                        (last_pos.lat() - pos.lat()).abs() > 0.0001 ||
                        (last_pos.lon() - pos.lon()).abs() > 0.0001
                }).unwrap_or(true);

                if add {
                    self.position_cache.push((pos, alt));
                }
            }
            self.cached_state = Some(state);
        }

        self.position_cache.clone()
    }

    pub fn last_position(&mut self) -> Option<(Position, f64, f32)> {
        self.last_position
    }
}

/// Map widget, created on each frame
pub struct Map<'a> {
    state: &'a mut MapState,
    vehicle_position: Option<(Position, f64, f32)>,
    vehicle_positions: Vec<(Position, f64)>,
}

impl<'a> Map<'a> {
    pub fn new(state: &'a mut MapState, data_source: &mut dyn DataSource) -> Self {
        let vehicle_positions = state.vehicle_positions(data_source);
        let vehicle_position = state.last_position();

        Self {
            state,
            vehicle_position,
            vehicle_positions,
        }
    }
}

impl<'a> Widget for Map<'a> {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        let rect = ui.max_rect();

        let tiles = match self.state.mapbox_tiles.as_mut() {
            Some(tiles) if self.state.satellite => tiles,
            _ => &mut self.state.osm_tiles
        };

        let detached_pos = self.state.memory.detached();

        let position = self.vehicle_position.map(|(pos, ..)| pos).unwrap_or(Position::from_lat_lon(49.861445, 8.68519));
        let gradient_lookup = self.state.gradient_lookup.clone();
        let mut map = walkers::Map::new(Some(tiles), &mut self.state.memory, position)
            .with_plugin(LinePlugin::new(vec![
                Line::PathWithValue(
                    self.vehicle_positions,
                    Box::new(move |val| {
                        let f = val / GRADIENT_MAX_ALT;
                        let i = (f * (gradient_lookup.len() as f64)) as usize;
                        Stroke { width: (1.0 + f*10.0) as f32, color: gradient_lookup[i] }
                    })
                )
            ]))
            .with_plugin(CrosshairPlugin {
                render_crosshair: detached_pos.is_some(),
            });

        // Label for current vehicle position
        if let Some((position, alt_agl, hdop)) = self.vehicle_position {
            map = map.with_plugin(Places::new(vec![
                Place {
                    position: position,
                    label: format!("{:.6}, {:.6}\nAGL: {:.1}m\nHDOP: {:.2}", position.lat(), position.lon(), alt_agl, hdop),
                    symbol: '🚀',
                    style: Style::default(),
                },
            ]));
        }

        let response = ui.add(map);

        // Panel for selecting map type
        let map_type_rect = Rect::from_two_pos(
            rect.left_bottom() + Vec2::new(10.0, -10.0),
            rect.left_bottom() + Vec2::new(100.0, -40.0)
        );
        ui.put(map_type_rect, |ui: &mut egui::Ui| {
            Frame::window(ui.style()).show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.state.satellite, false, "🗺");
                    ui.add_enabled_ui(self.state.mapbox_tiles.is_some(), |ui| {
                        ui.selectable_value(&mut self.state.satellite, true, "🌍")
                    });
                });
            }).response
        });

        // Panel for resetting map to vehicle position
        let reset_rect = Rect::from_two_pos(
            rect.right_bottom() + Vec2::new(-10.0, -10.0),
            rect.right_bottom() + Vec2::new(-40.0, -40.0)
        );
        ui.put(reset_rect, |ui: &mut egui::Ui| {
            Frame::window(ui.style()).show(ui, |ui| {
                ui.with_layout(Layout::right_to_left(egui::Align::Center), |ui| {
                    let detached_pos = self.state.memory.detached();
                    let pos = detached_pos.or(self.vehicle_position.map(|(p, ..)| p));
                    let coords = pos.map(|p| format!("{:.6},{:.6}", p.lat(), p.lon()));

                    ui.add_enabled_ui(detached_pos.is_some(), |ui| {
                        if ui.button("⌖").clicked() {
                            self.state.memory.follow_my_position();
                        }
                    });

                    ui.add_enabled_ui(coords.is_some(), |ui| {
                        if ui.button("📋").clicked() {
                            ui.output_mut(|o| o.copied_text = coords.clone().unwrap_or_default());
                        }
                    });

                    if detached_pos.is_some() {
                        ui.monospace(coords.unwrap_or_default());
                    }
                }).response
            }).response
        });

        // TODO: atribution

        response
    }
}
