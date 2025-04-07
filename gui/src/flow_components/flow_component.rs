use egui::{/*Color32,*/ Pos2, Rect, Ui};

// const EMPTY_COLOR: Color32 = Color32::LIGHT_GRAY;
// const N2_COLOR: Color32 = Color32::LIGHT_GREEN;
// const N2O_COLOR: Color32 = Color32::LIGHT_BLUE;

pub struct ResponseBounds{
    bounding_box: Rect,
    is_inside: Option<fn (point: Pos2) -> bool> 
}

pub struct DisplayValue{
    desc: String,
    unit: Option<String>,
    val: Option<f32>,
    //status: Option<fn () -> Status>
}

pub struct ComponentInfo{
    name: String,
    data: Vec<DisplayValue>
}

pub trait ComponentPainter{
    fn paint(&self, ui: &mut Ui) -> ResponseBounds;
}

pub struct FlowComponent{
    info: ComponentInfo,
    painter: Box<dyn ComponentPainter>,
    click_response: Option<String>
}


impl DisplayValue{

    pub fn new(desc: String, unit: Option<String>, val: Option<f32>) -> Self {
        Self { desc, unit, val }
    }

}

impl ComponentInfo{

    pub fn new(name: String, data: Vec<DisplayValue>) -> Self {
        Self { name, data }
    }

    pub fn draw(&self, ui: &mut egui::Ui){
        ui.vertical(|ui| {
            ui.label(self.name.clone());
            ui.separator();
            for dp in &self.data {
                let val_str = dp.val.map(|v| format!("{0:.1}", v)).unwrap_or("N/A".to_string());
                let unit_str= dp.unit.clone().map(|u| format!("[{}]", u)).unwrap_or("".to_string());
                ui.label(format!("{} {unit_str}: {val_str}", dp.desc)).on_hover_ui(|ui| {
                    ui.vertical(|ui| {
                        ui.label("Here could be YOUR detailed information!");
                        let sin: egui_plot::PlotPoints = (0..1000).map(|i| {
                            let x = i as f64 * 0.01;
                            [x, x.sin()]
                        }).collect();
                        let line = egui_plot::Line::new(sin);
                        egui_plot::Plot::new("The history of YOUR value!").width(100.0).view_aspect(2.0).show(ui, |plot_ui| plot_ui.line(line))
                    });
                });
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

    pub fn new(info: ComponentInfo, painter: Box<dyn ComponentPainter>, click_response: Option<String>) -> Self {
        Self { info, painter, click_response }
    }

    pub fn draw_and_respond(&self, ui: &mut Ui){
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

        response.on_hover_ui(|ui| {
            let mouse_pos =ui.input(|i| i.pointer.interact_pos());
            if bounds.is_inside.is_none_or(|f| mouse_pos.is_some_and(|pos| f(pos))) {
                ui.style_mut().interaction.selectable_labels = true;
                self.info.draw(ui);
            }
        });
    }

}