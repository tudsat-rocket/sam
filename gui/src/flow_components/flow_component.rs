use std::fmt::{self, Display};

use egui::{Color32, Pos2, Rect, Sense, Ui, Vec2};

pub struct ResponseBounds {
    bounding_box: Rect,
    is_inside: Option<Box<dyn Fn(&Pos2) -> bool>>,
}

pub struct DisplayValue {
    desc: String,
    unit: Option<String>,
    val: JustifiedValue,
}

#[derive(Clone)]
pub struct Sensor {
    name: &'static str,
}

pub const TEST_SENSOR: Sensor = Sensor { name: "Test Sensor" };

#[derive(Clone)]
pub enum Justification {
    Measured(Sensor),
    Reasoned,
    Process,
    None,
}

impl Display for Justification {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Justification::Measured(sensor) => format!("Measured by {}", sensor.name),
                Justification::Reasoned => format!("Reasoned"),
                Justification::Process => format!("Process"),
                Justification::None => format!("None"),
            }
        )
    }
}

pub const JUSTIFICATION_MEASRURED_PATTERN: &'static [u8] =
    include_bytes!("./../../assets/textures/full_pattern_32x32.png").as_slice();
pub const JUSTIFICATION_REASONED_PATTERN: &'static [u8] =
    include_bytes!("./../../assets/textures/diagonal_pattern_32x32.png").as_slice();
pub const JUSTIFICATION_PROCESS_PATTERN: &'static [u8] =
    include_bytes!("./../../assets/textures/crosshatch_pattern_32x32.png").as_slice();
pub const JUSTIFICATION_NONE_PATTERN: &'static [u8] =
    include_bytes!("./../../assets/textures/dots_pattern_32x32.png").as_slice();

#[derive(Clone)]
pub enum Value {
    F32(f32),
    U32(u32),
}

impl Display for Value {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Value::F32(v) => format! {"{}", v},
                Value::U32(v) => format! {"{}", v},
            }
        )
    }
}

#[derive(Clone)]
pub struct JustifiedValue {
    pub value: Option<Value>,
    pub justification: Justification,
}

impl JustifiedValue {
    pub fn new(value: Option<Value>, justification: Justification) -> Self {
        Self { value, justification }
    }
}

pub struct FluidType {
    pub name: &'static str,
    pub color: Color32,
}

pub struct Fluid {
    pub fluid_type: FluidType,
    pub pressure: JustifiedValue,
}

impl Fluid {
    pub fn new(fluid_type: FluidType, pressure: JustifiedValue) -> Self {
        Self { fluid_type, pressure }
    }
}

pub struct ComponentInfo {
    name: String,
    data: Vec<DisplayValue>,
}

pub trait ComponentPainter {
    fn paint(&self, ui: &mut Ui) -> ResponseBounds;
}

pub struct FlowComponent {
    info: ComponentInfo,
    painter: Box<dyn ComponentPainter>,
    click_response: Option<String>,
}

impl DisplayValue {
    pub fn new(desc: String, unit: Option<String>, val: JustifiedValue) -> Self {
        Self { desc, unit, val }
    }
}

impl ComponentInfo {
    pub fn new(name: String, data: Vec<DisplayValue>) -> Self {
        Self { name, data }
    }

    pub fn draw(&self, ui: &mut egui::Ui) {
        ui.vertical(|ui| {
            ui.label(self.name.clone());
            ui.separator();
            for dp in &self.data {
                let val_str = dp.val.value.as_ref().map(|v| format!("{0:.1}", v)).unwrap_or("N/A".to_string());
                let unit_str = dp.unit.clone().map(|u| format!("[{}]", u)).unwrap_or("".to_string());
                let dp_value_response = ui.label(format!("{} {unit_str}: {val_str}", dp.desc));

                let dp_value_id = ui.make_persistent_id(dp.desc.clone());
                let is_hovered = dp_value_response.hovered();

                if is_hovered {
                    let dp_value_pos = Pos2::new(ui.max_rect().max.x, dp_value_response.interact_rect.min.y);
                    egui::show_tooltip_at(ui.ctx(), ui.layer_id(), dp_value_id, dp_value_pos, |ui| {
                        ui.vertical(|ui| {
                            ui.label(format!("Justification: {}", dp.val.justification));
                            let sin: egui_plot::PlotPoints = (0..1000)
                                .map(|i| {
                                    let x = i as f64 * 0.01;
                                    [x, x.sin()]
                                })
                                .collect();
                            let line = egui_plot::Line::new(sin);
                            egui_plot::Plot::new("The history of YOUR value!")
                                .width(100.0)
                                .view_aspect(2.0)
                                .show(ui, |plot_ui| plot_ui.line(line))
                        });
                    });
                }
            }
        });
    }
}

impl ResponseBounds {
    pub fn new(bounding_box: Rect, is_inside: Option<Box<dyn Fn(&Pos2) -> bool>>) -> Self {
        Self {
            bounding_box,
            is_inside,
        }
    }

    pub fn combine(self, other: ResponseBounds) -> Self {
        Self {
            bounding_box: self.bounding_box.union(other.bounding_box),
            is_inside: None, //TODO Hans: Not implemented yet
        }
    }
}

impl FlowComponent {
    pub fn new(info: ComponentInfo, painter: Box<dyn ComponentPainter>, click_response: Option<String>) -> Self {
        Self {
            info,
            painter,
            click_response,
        }
    }

    pub fn draw_and_respond(&self, ui: &mut Ui) {
        let bounds = self.painter.paint(ui);
        let response = ui.interact(bounds.bounding_box, egui::Id::new(self.info.name.clone()), egui::Sense::click());
        match &self.click_response {
            Some(s) => {
                if response.clicked() {
                    println!("{s}");
                    // let modal = Modal::new(Id::new("ConfirmModal")).show(ui.ctx(), |ui | {
                    //     ui.label(self.click_response.clone().expect("Unclickable component was clicked!"));
                    // });
                }
            }
            _ => {
                ();
            }
        }

        let mouse_pos = ui.input(|i| i.pointer.interact_pos());
        let tooltip_id = ui.make_persistent_id(self.info.name.clone());
        let is_hovered = response.hovered() && bounds.is_inside.is_none_or(|f| mouse_pos.is_some_and(|pos| f(&pos)));
        let is_tooltip_open = ui.ctx().data_mut(|d| d.get_temp::<bool>(tooltip_id)).unwrap_or(false);

        if is_hovered || is_tooltip_open {
            let tooltip_pos = bounds.bounding_box.min + bounds.bounding_box.size() * Vec2::new(0.75, 0.75);
            egui::show_tooltip_at(ui.ctx(), ui.layer_id(), tooltip_id, tooltip_pos, |ui| {
                ui.style_mut().interaction.selectable_labels = true;
                self.info.draw(ui);
                let tooltip_response = ui.interact(ui.max_rect(), tooltip_id, Sense::hover());
                let is_tooltip_hovered = tooltip_response.hovered();
                //The entry in the IdTypeMap is never deleted, we could do that if necessary
                ui.ctx().data_mut(|d| d.insert_temp(tooltip_id, is_hovered || is_tooltip_hovered));
            });
        }
    }
}
