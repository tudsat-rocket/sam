//! Contains our map widget, based on the walkers crate.

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use directories::ProjectDirs;
use eframe::egui;
use egui::{Color32, Vec2, Widget, Frame, Rect, Stroke, Layout};
use walkers::extras::{Places, Place, Style};
use walkers::{Tiles, MapMemory, Position, Plugin, HttpOptions};
use nalgebra::Vector3;

use shared_types::telemetry::FlightMode;

use crate::data_source::DataSource;
use crate::telemetry_ext::ColorExt;

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

pub struct Path<'a, T> {
    values: &'a Vec<(Position, T)>,
    stroke_callback: Box<dyn Fn(&T) -> egui::Stroke>,
}

pub struct PathPlugin<'a, T> {
    paths: Vec<Path<'a, T>>
}

impl<'a, T> PathPlugin<'a, T> {
    pub fn new(paths: Vec<Path<'a, T>>) -> Self {
        Self { paths }
    }
}

impl<'a, T> Plugin for PathPlugin<'a, T> {
    fn run(&mut self, _response: &egui::Response, painter: egui::Painter, projector: &walkers::Projector) {
        for p in &self.paths {
            let screen_positions: Vec<_> = p.values.iter()
                .map(|(p, val)| (projector.project(*p).to_pos2(), val))
                .collect();
            for segment in screen_positions.windows(2) {
                painter.line_segment([segment[0].0, segment[1].0], (p.stroke_callback)(segment[0].1));
            }
        }
    }
}

#[derive(PartialEq, Clone, Copy)]
enum PositionSource {
    Estimate,
    Gps
}

#[derive(PartialEq, Clone, Copy)]
enum Visualization {
    Altitude,
    FlightMode,
    Attitude,
    Uncertainty,
}

/// Permanent Map data store, kept in memory the entire time
pub struct MapState {
    osm_tiles: Tiles,
    mapbox_tiles: Option<Tiles>,
    memory: MapMemory,
    satellite: bool,
    position_source: PositionSource,
    visualization: Visualization,
    gradient_lookup: Vec<Color32>,
    estimated_positions: Vec<(Position, (f64, Vector3<f32>, FlightMode, f32))>,
    gps_positions: Vec<(Position, (f64, Vector3<f32>, FlightMode, f32))>,
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
            ..Default::default()
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
            ctx.to_owned()
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
            position_source: PositionSource::Estimate,
            visualization: Visualization::Altitude,
            gradient_lookup,
            estimated_positions: Vec::new(),
            gps_positions: Vec::new(),
            cached_state: None,
        }
    }

    pub fn vehicle_positions(&mut self, data_source: &mut dyn DataSource) -> Vec<(Position, (f64, Vector3<f32>, FlightMode, f32))> {
        let Some(last) = data_source.vehicle_states().next_back() else {
            return Vec::new();
        };

        let state = (last.0, data_source.vehicle_states().len());

        // repopulate cache
        if self.cached_state.map(|s| s != state).unwrap_or(true) {
            let all_estimated_positions = data_source.vehicle_states()
                .scan((None, None, None, None), |(ground_asl, altitude_asl, orientation, fm), (_, vs)| {
                    *ground_asl = vs.altitude_ground_asl.or(*ground_asl);
                    *altitude_asl = vs.altitude_asl.or(*altitude_asl);
                    *orientation = vs.orientation.or(*orientation);
                    *fm = vs.mode.or(*fm);
                    Some((*ground_asl, *altitude_asl, *orientation, *fm, vs))
                })
                .filter(|(_, _, _, _, vs)| vs.latitude.is_some() && vs.longitude.is_some())
                .map(|(ground_asl, alt_asl, orientation, fm, vs)| {
                    let pos = Position::from_lat_lon(vs.latitude.unwrap() as f64, vs.longitude.unwrap() as f64);
                    let alt = (alt_asl.unwrap_or_default() - ground_asl.unwrap_or_default()) as f64;
                    let att = orientation.unwrap_or_default() * Vector3::new(0.0, 0.0, 1.0);
                    let variance = vs.sim.as_ref().map(|sim| sim.kalman_P[0]).unwrap_or(0.0);
                    (pos, alt, att, fm.unwrap_or_default(), variance)
                });
            let all_gps_positions = data_source.vehicle_states()
                .scan((None, None, None, None), |(ground_asl, altitude_asl, orientation, fm), (_, vs)| {
                    *ground_asl = vs.altitude_ground_asl.or(*ground_asl);
                    *altitude_asl = vs.gps.as_ref().and_then(|gps| gps.altitude).or(*altitude_asl);
                    *orientation = vs.orientation.or(*orientation);
                    *fm = vs.mode.or(*fm);
                    Some((*ground_asl, *altitude_asl, *orientation, *fm, vs))
                })
                .filter(|(_, _, _, _, vs)| vs.gps.as_ref().map(|gps|
                    gps.latitude.is_some() && gps.longitude.is_some() && gps.num_satellites >= 6 && gps.hdop < 500
                ).unwrap_or(false))
                .map(|(ground_asl, alt_asl, orientation, fm, vs)| {
                    let pos = Position::from_lat_lon(
                        vs.gps.as_ref().and_then(|gps| gps.latitude).unwrap() as f64,
                        vs.gps.as_ref().and_then(|gps| gps.longitude).unwrap() as f64
                    );
                    let alt = (alt_asl.unwrap_or_default() - ground_asl.unwrap_or_default()) as f64;
                    let att = orientation.unwrap_or_default() * Vector3::new(0.0, 0.0, 1.0);
                    let hdop = vs.gps.as_ref().map(|gps| gps.hdop).unwrap_or_default() as f32 / 100.0;
                    (pos, alt, att, fm.unwrap_or_default(), hdop)
                });

            self.estimated_positions.truncate(0);
            for (pos, alt, att, fm, variance) in all_estimated_positions {
                let add = self.estimated_positions.last().map(|(last_pos, (last_alt, _att, _fm, _var))| {
                    (last_alt - alt).abs() > 20.0 ||
                        (last_pos.lat() - pos.lat()).abs() > 0.00001 ||
                        (last_pos.lon() - pos.lon()).abs() > 0.00001
                }).unwrap_or(true);

                if add {
                    self.estimated_positions.push((pos, (alt, att, fm, variance)));
                }
            }

            self.gps_positions.truncate(0);
            for (pos, alt, att, fm, hdop) in all_gps_positions {
                let add = self.gps_positions.last().map(|(last_pos, (last_alt, _att, _fm, _hdop))| {
                    (last_alt - alt).abs() > 20.0 ||
                        (last_pos.lat() - pos.lat()).abs() > 0.00001 ||
                        (last_pos.lon() - pos.lon()).abs() > 0.00001
                }).unwrap_or(true);

                if add {
                    self.gps_positions.push((pos, (alt, att, fm, hdop)));
                }
            }

            self.cached_state = Some(state);
        }

        match self.position_source {
            PositionSource::Estimate => self.estimated_positions.clone(),
            PositionSource::Gps => self.gps_positions.clone(),
        }
    }

    pub fn last_position(&mut self) -> Option<(Position, (f64, Vector3<f32>, FlightMode, f32))> {
        match self.position_source {
            PositionSource::Estimate => self.estimated_positions.last().copied(),
            PositionSource::Gps => self.gps_positions.last().copied(),
        }
    }
}

/// Map widget, created on each frame
pub struct Map<'a> {
    state: &'a mut MapState,
    vehicle_position: Option<(Position, (f64, Vector3<f32>, FlightMode, f32))>,
    vehicle_positions: Vec<(Position, (f64, Vector3<f32>, FlightMode, f32))>,
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
        let pos_source = self.state.position_source;

        let vis_plugin = match self.state.visualization {
            Visualization::Altitude => Path {
                values: &self.vehicle_positions,
                stroke_callback: Box::new(move |(alt, _att, _fm, _var)| {
                    let f = alt / GRADIENT_MAX_ALT;
                    let i = (f * (gradient_lookup.len() as f64)) as usize;
                    Stroke { width: (1.0 + (alt/GRADIENT_MAX_ALT)*10.0) as f32, color: gradient_lookup[usize::min(i, gradient_lookup.len() - 1)] }
                })
            },
            Visualization::FlightMode => Path {
                values: &self.vehicle_positions,
                stroke_callback: Box::new(move |(alt, _att, fm, _var)| {
                    Stroke { width: (1.0 + (alt / GRADIENT_MAX_ALT)*10.0) as f32, color: fm.color() }
                })
            },
            Visualization::Attitude => Path {
                values: &self.vehicle_positions,
                stroke_callback: Box::new(move |(alt, att, _fm, _var)| {
                    let color = Color32::from_rgb(
                        (256.0 * (att.x + 1.0) / 2.0) as u8,
                        (256.0 * (att.y + 1.0) / 2.0) as u8,
                        (256.0 * (att.z + 1.0) / 2.0) as u8,
                    );
                    Stroke { width: (1.0 + (alt/GRADIENT_MAX_ALT)*10.0) as f32, color }
                })
            },
            Visualization::Uncertainty => Path {
                values: &self.vehicle_positions,
                stroke_callback: Box::new(move |(alt, _att, _fm, var)| {
                    let f = if pos_source == PositionSource::Estimate {
                        f32::min(*var, 5.0) / 5.0
                    } else {
                        var / 10.00
                    };
                    let i = ((0.3 - f64::min(f as f64, 1.0) * 0.3) * (gradient_lookup.len() as f64)) as usize;
                    Stroke {
                        width: (1.0 + (alt/GRADIENT_MAX_ALT)*10.0) as f32,
                        color: gradient_lookup[usize::min(i, gradient_lookup.len() - 1)]
                    }
                })
            }
        };

        let mut map = walkers::Map::new(Some(tiles), &mut self.state.memory, position)
            .with_plugin(PathPlugin::new(vec![vis_plugin]))
            .with_plugin(CrosshairPlugin {
                render_crosshair: detached_pos.is_some(),
            });

        // Label for current vehicle position
        if let Some((position, (alt_agl, _att, _fm, hdop))) = self.vehicle_positions.last().as_ref().copied() {
            map = map.with_plugin(Places::new(vec![
                Place {
                    position: *position,
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

        // Panel for selecting path visualizations
        let map_type_rect = Rect::from_two_pos(
            rect.left_top() + Vec2::new(10.0, 10.0),
            rect.left_top() + Vec2::new(100.0, 40.0)
        );
        ui.put(map_type_rect, |ui: &mut egui::Ui| {
            Frame::window(ui.style()).show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.state.position_source, PositionSource::Estimate, "🗠");
                    ui.selectable_value(&mut self.state.position_source, PositionSource::Gps, "🌍");
                    ui.separator();
                    ui.selectable_value(&mut self.state.visualization, Visualization::Altitude, "⬍");
                    ui.selectable_value(&mut self.state.visualization, Visualization::FlightMode, "🏷");
                    ui.selectable_value(&mut self.state.visualization, Visualization::Attitude, "🔃");
                    ui.selectable_value(&mut self.state.visualization, Visualization::Uncertainty, "⁉");
                });
            }).response
        });

        // TODO: attribution

        response
    }
}
