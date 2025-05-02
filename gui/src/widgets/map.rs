//! Contains our map widget, based on the walkers crate.

use directories::ProjectDirs;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use eframe::egui;
use egui::{Color32, Frame, Layout, Rect, Stroke, Ui, Vec2, Widget};
use transform_gizmo_egui::math::{DMat4, DVec3, Transform};
use transform_gizmo_egui::{Gizmo, GizmoConfig, GizmoExt, GizmoMode, GizmoVisuals};
use walkers::extras::{Place, Places, Style};
use walkers::{HttpOptions, HttpTiles, MapMemory, Plugin, Position, Projector};

use shared_types::telemetry::FlightMode;
use telemetry::{Dim, Metric};

use crate::settings::AppSettings;
use crate::utils::telemetry_ext::ColorExt;
use crate::Backend;

const GRADIENT_MAX_ALT: f64 = 10000.0;

pub struct CrosshairPlugin {
    render_crosshair: bool,
}

impl Plugin for CrosshairPlugin {
    fn run(self: Box<Self>, ui: &mut Ui, _response: &egui::Response, _projector: &walkers::Projector) {
        let rect = ui.painter().clip_rect();
        let crosshair_stroke = Stroke {
            width: 1.0,
            color: Color32::GRAY.gamma_multiply(0.8),
        };

        if self.render_crosshair {
            ui.painter().line_segment([rect.center_top(), rect.center_bottom()], crosshair_stroke);
            ui.painter().line_segment([rect.left_center(), rect.right_center()], crosshair_stroke);
        }
    }
}

pub struct Path<'a, T> {
    values: &'a Vec<(Position, T)>,
    stroke_callback: Box<dyn Fn(&T) -> egui::Stroke>,
}

pub struct PathPlugin<'a, T> {
    paths: Vec<Path<'a, T>>,
}

impl<'a, T> PathPlugin<'a, T> {
    pub fn new(paths: Vec<Path<'a, T>>) -> Self {
        Self { paths }
    }
}

impl<'a, T> Plugin for PathPlugin<'a, T> {
    fn run(self: Box<Self>, ui: &mut Ui, _response: &egui::Response, projector: &walkers::Projector) {
        for p in &self.paths {
            let screen_positions: Vec<_> =
                p.values.iter().map(|(p, val)| (projector.project(*p).to_pos2(), val)).collect();
            for segment in screen_positions.windows(2) {
                ui.painter().line_segment([segment[0].0, segment[1].0], (p.stroke_callback)(segment[0].1));
            }
        }
    }
}

#[derive(PartialEq, Clone, Copy)]
enum PositionSource {
    Estimate,
    Gps,
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
    osm_tiles: HttpTiles,
    mapbox_tiles: Option<HttpTiles>,
    memory: MapMemory,
    satellite: bool,
    position_source: PositionSource,
    visualization: Visualization,
    show_gizmos: bool,
    gradient_lookup: Vec<Color32>,
    estimated_positions: Vec<(Position, (f64, Vector3<f32>, FlightMode, f32))>,
    gps_positions: Vec<(Position, (f64, Vector3<f32>, FlightMode, f32))>,
    cached_state: Option<(f64, usize)>,
}

impl MapState {
    fn http_options() -> HttpOptions {
        // We don't cache anything on web assembly
        #[cfg(target_arch = "wasm32")]
        let cache_path = None;

        // On Android, we just hardcode the path for now. If we wanted to do it properly, we'd
        // have to request a path and pass it to our code via the JNI.
        #[cfg(target_os = "android")]
        let cache_path = Some(std::path::PathBuf::from("/data/user/0/space.tudsat.sam/cache"));

        // On other platforms, we store map tiles on-disk
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        let cache_path = Some(ProjectDirs::from("space", "tudsat", "sam").unwrap().cache_dir().into());

        HttpOptions {
            cache: cache_path,
            ..Default::default()
        }
    }

    pub fn new(ctx: &egui::Context, mapbox_access_token: Option<String>) -> Self {
        let osm_tiles = HttpTiles::with_options(walkers::sources::OpenStreetMap, Self::http_options(), ctx.to_owned());

        // We only show the mapbox map if we have an access token
        let mapbox_access_token = mapbox_access_token.or(option_env!("MAPBOX_ACCESS_TOKEN").map(|s| s.to_string()));
        let mapbox_tiles = mapbox_access_token.map(|t| {
            HttpTiles::with_options(
                walkers::sources::Mapbox {
                    style: walkers::sources::MapboxStyle::Satellite,
                    access_token: t.to_string(),
                    high_resolution: true,
                },
                Self::http_options(),
                ctx.to_owned(),
            )
        });

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
            position_source: PositionSource::Gps,
            visualization: Visualization::Altitude,
            show_gizmos: true,
            gradient_lookup,
            estimated_positions: Vec::new(),
            gps_positions: Vec::new(),
            cached_state: None,
        }
    }

    pub fn vehicle_positions(
        &mut self,
        backend: &mut Backend,
    ) -> Vec<(Position, (f64, Vector3<f32>, FlightMode, f32))> {
        let Some(last) = backend.end() else {
            return Vec::new();
        };

        let state =
            (last, backend.timeseries(&Metric::PositionWorldSpace(Dim::Z)).map(|t| t.len()).unwrap_or_default());

        // repopulate cache
        if self.cached_state.map(|s| s != state).unwrap_or(true) {
            let all_estimated_positions = backend
                .zip_timeseries(
                    [
                        Metric::Latitude,
                        Metric::Longitude,
                        Metric::PositionWorldSpace(Dim::Z),
                        Metric::GroundAltitudeASL,
                        Metric::Orientation(0),
                        Metric::Orientation(1),
                        Metric::Orientation(2),
                        Metric::Orientation(3),
                        Metric::FlightMode,
                        Metric::KalmanStateCovariance(0, 0),
                    ],
                    1.0,
                )
                .filter(|(_x, [lat, lon, ..])| lat.is_some() && lon.is_some())
                .map(|(_x, [lat, lon, alt_asl, ground_asl, q0, q1, q2, q3, fm, var])| {
                    let q: Vec<f64> =
                        [q0, q1, q2, q3].iter().copied().collect::<Option<_>>().unwrap_or(vec![1.0, 0.0, 0.0, 0.0]);
                    let orientation =
                        UnitQuaternion::from_quaternion(Quaternion::from_parts(q[3], Vector3::new(q[0], q[1], q[2])))
                            .cast::<f32>();
                    let pos = Position::new(lon.unwrap_or_default(), lat.unwrap_or_default());
                    let alt = alt_asl.unwrap_or_default() - ground_asl.unwrap_or_default();
                    let att = orientation * Vector3::new(0.0, 0.0, 1.0);
                    (pos, alt, att, (fm.unwrap_or_default() as u8).try_into().unwrap(), var.unwrap_or_default() as f32)
                    // TODO: fm is very hacky
                });

            let all_gps_positions = backend
                .zip_timeseries(
                    [
                        Metric::GpsLatitude,
                        Metric::GpsLongitude,
                        Metric::GpsAltitude,
                        Metric::GroundAltitudeASL,
                        Metric::Orientation(0),
                        Metric::Orientation(1),
                        Metric::Orientation(2),
                        Metric::Orientation(3),
                        Metric::FlightMode,
                        Metric::GpsHdop,
                        Metric::GpsSatellites,
                    ],
                    10.0,
                )
                .filter(|(_x, [lat, lon, _alt_asl, _ground_asl, _q0, _q1, _q2, _q3, _fm, hdop, sats])| {
                    lat.is_some() && lon.is_some() && sats.unwrap_or(0.0) >= 6.0 && hdop.unwrap_or(99.0) < 5.0
                })
                .map(|(_x, [lat, lon, alt_asl, ground_asl, q0, q1, q2, q3, fm, hdop, _sats])| {
                    let q: Vec<f64> =
                        [q0, q1, q2, q3].iter().copied().collect::<Option<_>>().unwrap_or(vec![1.0, 0.0, 0.0, 0.0]);
                    let orientation =
                        UnitQuaternion::from_quaternion(Quaternion::from_parts(q[3], Vector3::new(q[0], q[1], q[2])))
                            .cast::<f32>();
                    let pos = Position::new(lon.unwrap_or_default(), lat.unwrap_or_default());
                    let alt = alt_asl.unwrap_or_default() - ground_asl.unwrap_or_default();
                    let att = orientation * Vector3::new(0.0, 0.0, 1.0);
                    (pos, alt, att, (fm.unwrap_or_default() as u8).try_into().unwrap(), hdop.unwrap_or_default() as f32)
                    // TODO: fm is very hacky
                });

            self.estimated_positions.truncate(0);
            for (pos, alt, att, fm, variance) in all_estimated_positions {
                let add = self
                    .estimated_positions
                    .last()
                    .map(|(last_pos, (last_alt, _att, _fm, _var))| {
                        (last_alt - alt).abs() > 20.0
                            || (last_pos.y() - pos.y()).abs() > 0.00001
                            || (last_pos.x() - pos.x()).abs() > 0.00001
                    })
                    .unwrap_or(true);

                if add {
                    self.estimated_positions.push((pos, (alt, att, fm, variance)));
                }
            }

            self.gps_positions.truncate(0);
            for (pos, alt, att, fm, hdop) in all_gps_positions {
                let add = self
                    .gps_positions
                    .last()
                    .map(|(last_pos, (last_alt, _att, _fm, _hdop))| {
                        (last_alt - alt).abs() > 20.0
                            || (last_pos.y() - pos.y()).abs() > 0.00001
                            || (last_pos.x() - pos.x()).abs() > 0.00001
                    })
                    .unwrap_or(true);

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
    orientation: Option<UnitQuaternion<f32>>,
    settings: &'a AppSettings,
}

impl<'a> Map<'a> {
    pub fn new(state: &'a mut MapState, backend: &mut Backend, settings: &'a AppSettings) -> Self {
        let vehicle_positions = state.vehicle_positions(backend);
        let vehicle_position = state.last_position();

        let q0 = backend.current_value(Metric::Orientation(0));
        let q1 = backend.current_value(Metric::Orientation(1));
        let q2 = backend.current_value(Metric::Orientation(2));
        let q3 = backend.current_value(Metric::Orientation(3));
        let orientation = match (q0, q1, q2, q3) {
            (Some(q0), Some(q1), Some(q2), Some(q3)) => {
                let orientation =
                    UnitQuaternion::from_quaternion(Quaternion::from_parts(q3, Vector3::new(q0, q1, q2))).cast::<f32>();
                Some(orientation)
            }
            _ => None,
        };

        Self {
            state,
            vehicle_position,
            vehicle_positions,
            orientation,
            settings,
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
            _ => &mut self.state.osm_tiles,
        };

        let detached_pos = self.state.memory.detached();

        let position = self.vehicle_position.map(|(pos, ..)| pos).unwrap_or(Position::new(8.68519, 49.861445));
        let gradient_lookup = self.state.gradient_lookup.clone();
        let pos_source = self.state.position_source;

        let vis_plugin = match self.state.visualization {
            Visualization::Altitude => Path {
                values: &self.vehicle_positions,
                stroke_callback: Box::new(move |(alt, _att, _fm, _var)| {
                    let f = alt / GRADIENT_MAX_ALT;
                    let i = (f * (gradient_lookup.len() as f64)) as usize;
                    Stroke {
                        width: (1.0 + (alt / GRADIENT_MAX_ALT) * 10.0) as f32,
                        color: gradient_lookup[usize::min(i, gradient_lookup.len() - 1)],
                    }
                }),
            },
            Visualization::FlightMode => Path {
                values: &self.vehicle_positions,
                stroke_callback: Box::new(move |(alt, _att, fm, _var)| Stroke {
                    width: (1.0 + (alt / GRADIENT_MAX_ALT) * 10.0) as f32,
                    color: fm.color(),
                }),
            },
            Visualization::Attitude => Path {
                values: &self.vehicle_positions,
                stroke_callback: Box::new(move |(alt, att, _fm, _var)| {
                    let color = Color32::from_rgb(
                        (256.0 * (att.x + 1.0) / 2.0) as u8,
                        (256.0 * (att.y + 1.0) / 2.0) as u8,
                        (256.0 * (att.z + 1.0) / 2.0) as u8,
                    );
                    Stroke {
                        width: (1.0 + (alt / GRADIENT_MAX_ALT) * 10.0) as f32,
                        color,
                    }
                }),
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
                        width: (1.0 + (alt / GRADIENT_MAX_ALT) * 10.0) as f32,
                        color: gradient_lookup[usize::min(i, gradient_lookup.len() - 1)],
                    }
                }),
            },
        };

        let mut map = walkers::Map::new(Some(tiles), &mut self.state.memory, position)
            .with_plugin(PathPlugin::new(vec![vis_plugin]))
            .with_plugin(CrosshairPlugin {
                render_crosshair: detached_pos.is_some(),
            });

        if !self.state.show_gizmos {
            if let Some((position, (alt_agl, _att, _fm, hdop))) = self.vehicle_positions.last().as_ref().copied() {
                map = map.with_plugin(Places::new(vec![Place {
                    position: *position,
                    label: format!("{:.6}, {:.6}\nAGL: {:.1}m\nHDOP: {:.2}", position.y(), position.x(), alt_agl, hdop),
                    symbol: '🚀',
                    style: Style::default(),
                }]));
            }
        }

        let (gcs_lat, gcs_lon) = self.settings.ground_station_position.unwrap_or_default();
        let gcs_position = Position::new(gcs_lon, gcs_lat);
        let attitude_text =
            if let Some((position, (alt_agl, _att, _fm, _hdop))) = self.vehicle_positions.last().as_ref().copied() {
                let (rel_lng, rel_lat) = (position.x() - gcs_position.x(), position.y() - gcs_position.y());
                let (rel_x, rel_y) = (rel_lng * 111_111.0 * gcs_position.y().to_radians().cos(), rel_lat * 111_111.0);
                let ground_dist = (rel_x.powi(2) + rel_y.powi(2)).sqrt();
                let azimuth = rel_x.atan2(rel_y).to_degrees();
                let elevation = alt_agl.atan2(ground_dist).to_degrees();
                Some(format!("\nAzim.: {:.1}°\nElev.: {:.1}°", azimuth, elevation))
            } else {
                None
            };

        map = map.with_plugin(Places::new(vec![Place {
            position: gcs_position,
            label: format!("GCS{}", attitude_text.unwrap_or_default()),
            symbol: '📡',
            style: Style::default(),
        }]));

        let response = ui.add(map);

        if self.state.show_gizmos {
            if let Some(q) = self.orientation {
                let viewport = ui.clip_rect();

                // Fun type conversion bullshit
                let rotation: mint::Quaternion<f64> = q.cast::<f64>().into();
                let rotation: transform_gizmo_egui::mint::Quaternion<f64> = rotation.into();

                let view_matrix = DMat4::look_at_rh(DVec3::new(0., 0., 1.), DVec3::ZERO, DVec3::new(0., 1., 0.));
                let projection_matrix = DMat4::orthographic_rh(
                    viewport.left() as f64,
                    viewport.right() as f64,
                    -viewport.bottom() as f64,
                    -viewport.top() as f64,
                    0.1,
                    1000.0,
                );

                // We use viewport pixel coordinates (obtained from the Map projector)
                // for the rendering of the gizmo, but we need to invert the y axis,
                // since screen coordinates are Y down
                let projector = Projector::new(viewport, &self.state.memory, position);
                let viewport_pos = projector.project(position);
                let translation = DVec3::new(viewport_pos.x as f64, -viewport_pos.y as f64, 0.0);
                let transform = Transform::from_scale_rotation_translation(DVec3::ONE, rotation, translation);

                let visuals = GizmoVisuals {
                    inactive_alpha: 1.0,
                    highlight_alpha: 1.0,
                    gizmo_size: 50.0,
                    ..Default::default()
                };

                let config = GizmoConfig {
                    viewport,
                    view_matrix: view_matrix.into(),
                    projection_matrix: projection_matrix.into(),
                    modes: GizmoMode::all_translate(),
                    orientation: transform_gizmo_egui::GizmoOrientation::Local,
                    visuals,
                    ..Default::default()
                };

                Gizmo::new(config).interact(ui, &[transform]);
            }
        }

        // Panel for selecting map type
        let map_type_rect = Rect::from_two_pos(
            rect.left_bottom() + Vec2::new(10.0, -10.0),
            rect.left_bottom() + Vec2::new(100.0, -40.0),
        );
        ui.put(map_type_rect, |ui: &mut egui::Ui| {
            Frame::window(ui.style())
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut self.state.satellite, false, "🗺");
                        ui.add_enabled_ui(self.state.mapbox_tiles.is_some(), |ui| {
                            ui.selectable_value(&mut self.state.satellite, true, "🌍")
                        });
                    });
                })
                .response
        });

        // Panel for resetting map to vehicle position
        let reset_rect = Rect::from_two_pos(
            rect.right_bottom() + Vec2::new(-10.0, -10.0),
            rect.right_bottom() + Vec2::new(-40.0, -40.0),
        );
        ui.put(reset_rect, |ui: &mut egui::Ui| {
            Frame::window(ui.style())
                .show(ui, |ui| {
                    ui.with_layout(Layout::right_to_left(egui::Align::Center), |ui| {
                        let detached_pos = self.state.memory.detached();
                        let pos = detached_pos.or(self.vehicle_position.map(|(p, ..)| p));
                        let coords = pos.map(|p| format!("{:.6},{:.6}", p.y(), p.x()));

                        ui.add_enabled_ui(detached_pos.is_some(), |ui| {
                            if ui.button("⌖").clicked() {
                                self.state.memory.follow_my_position();
                            }
                        });

                        ui.add_enabled_ui(coords.is_some(), |ui| {
                            if ui.button("📋").clicked() {
                                ui.ctx().copy_text(coords.clone().unwrap_or_default());
                            }
                        });

                        if detached_pos.is_some() {
                            ui.monospace(coords.unwrap_or_default());
                        }
                    })
                    .response
                })
                .response
        });

        // Panel for selecting path visualizations
        let map_type_rect =
            Rect::from_two_pos(rect.left_top() + Vec2::new(10.0, 10.0), rect.left_top() + Vec2::new(100.0, 40.0));
        ui.put(map_type_rect, |ui: &mut egui::Ui| {
            Frame::window(ui.style())
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut self.state.position_source, PositionSource::Estimate, "🗠");
                        ui.selectable_value(&mut self.state.position_source, PositionSource::Gps, "🌍");
                        ui.separator();
                        ui.selectable_value(&mut self.state.visualization, Visualization::Altitude, "⬍");
                        ui.selectable_value(&mut self.state.visualization, Visualization::FlightMode, "🏷");
                        ui.selectable_value(&mut self.state.visualization, Visualization::Attitude, "🔃");
                        ui.selectable_value(&mut self.state.visualization, Visualization::Uncertainty, "⁉");
                    });
                })
                .response
        });

        // Panel for switching between gizmos and position tags
        let gizmo_rect =
            Rect::from_two_pos(rect.right_top() + Vec2::new(-10.0, 10.0), rect.right_top() + Vec2::new(-100.0, 40.0));
        ui.put(gizmo_rect, |ui: &mut egui::Ui| {
            Frame::window(ui.style())
                .show(ui, |ui| {
                    ui.with_layout(Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.selectable_value(&mut self.state.show_gizmos, false, "📋");
                        ui.selectable_value(&mut self.state.show_gizmos, true, "🔃");
                    });
                })
                .response
        });

        // TODO: attribution

        response
    }
}
