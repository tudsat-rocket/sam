// enum Status{
//     NOMINAL,
//     CONCERNING,
//     DANGEROUS,
// }

use egui::{Pos2, Rect, Ui};

pub struct ResponseBounds{
    bounding_box: Rect,
    is_inside: Option<fn (point: Pos2) -> bool> 
}

pub struct DataPoint{
    desc: String,
    unit: Option<String>,
    val: Option<f32>,
    //status: Option<fn () -> Status>
}

// pub enum ComponentState{
//     FillState(f32),
//     OtherTestState(i32)
// }

pub struct ComponentInfo{
    name: String,
    data: Vec<DataPoint>
}

//pub trait ComponentState{
//    fn getState
//}

pub trait ComponentPainter{
    fn paint(&self, ui: &mut Ui) -> ResponseBounds;
}

pub struct FlowComponent{
    info: ComponentInfo,
    //state: Box<dyn ComponentState>
    painter: Box<dyn ComponentPainter>
}

//let p = TankPainter: ComponentPainter
//let s = TankState: ComponentState
//let i = ComponentInfo => Additional info, Type agnostic
//p.paint(ui, s) wobei p und s vom Typ Tank sein müsse
//i.show()

//Ideally: TankState und Component Info iwie zusammen
//Idea 1: Via dict{desc, ...}

impl DataPoint{

    pub fn new(desc: String, unit: Option<String>, val: Option<f32>) -> Self {
        Self { desc, unit, val }
    }

}

impl ComponentInfo{

    pub fn new(name: String, data: Vec<DataPoint>) -> Self {
        Self { name, data }
    }

    pub fn draw(&self, ui: &mut egui::Ui){
        ui.vertical(|ui| {
            ui.label(self.name.clone());
            ui.separator();
            for dp in &self.data {
                let val_str = dp.val.map(|v| format!("{0:.1}", v)).unwrap_or("N/A".to_string());
                let unit_str= dp.unit.clone().map(|u| format!("[{}]", u)).unwrap_or("".to_string());
                ui.label(format!("{} {unit_str}: {val_str}", dp.desc));
            }
        });
    }

}


impl ResponseBounds{

    pub fn new(bounding_box: Rect, is_inside: Option<fn(point: Pos2) -> bool>) -> Self {
        Self{ bounding_box, is_inside }
    }

}

impl FlowComponent {

    pub fn new(info: ComponentInfo, painter: Box<dyn ComponentPainter>) -> Self {
        Self { info, painter }
    }

    pub fn draw_and_respond(&self, ui: &mut Ui){
        let bounds = self.painter.paint(ui);
        ui.interact(bounds.bounding_box, egui::Id::new(self.info.name.clone()), egui::Sense::hover()).on_hover_ui(|ui| {
            let mouse_pos =ui.input(|i| i.pointer.interact_pos());
            if bounds.is_inside.is_none_or(|f| mouse_pos.is_some_and(|pos| f(pos))) {
                ui.style_mut().interaction.selectable_labels = true;
                self.info.draw(ui);
            }
        });
    }

}