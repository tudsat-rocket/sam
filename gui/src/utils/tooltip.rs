use std::time::{Duration, Instant};

use crate::system_diagram_components::core::constants::{GAMMA_MUL_ON_HOVER, TOOLTIP_DURATION};

#[derive(Clone, Copy)]
struct ActiveTooltip {
    tooltip_id: u32,
    last_hover: Instant,
}

impl ActiveTooltip {

    fn new(tooltip_id: u32,  last_hover: Instant) -> Self {
        Self { tooltip_id, last_hover }
    }

}

pub struct TooltipManager {
    manager_id: egui::Id,
    ///Specifies the tooltip ID of the active tooltip for a given layer ID
    active_tooltips: Vec<ActiveTooltip>,
    layer_counter: usize,
    tooltip_counter: u32,
}

impl TooltipManager {

    pub fn new(ui: &mut egui::Ui) -> Self{
        let id = ui.make_persistent_id("Tooltip Manager");
        Self { 
            manager_id: id,
            active_tooltips: ui.ctx().data_mut(|id_type_map| id_type_map.get_temp::<Vec<ActiveTooltip>>(id)).unwrap_or(vec![]),
            layer_counter: 0,
            tooltip_counter: 0,
        }
    }

    pub fn save_state(self, ui: &mut egui::Ui) {
        let active_tooltips = self.active_tooltips
                                                    .iter()
                                                    .rev()
                                                    .scan(None, |max, cur| 
                                                        Some(ActiveTooltip::new(cur.tooltip_id, Instant::max(max.unwrap_or(cur.last_hover), cur.last_hover)))
                                                    )
                                                    .filter(|tooltip| tooltip.last_hover.elapsed().as_secs_f32() < TOOLTIP_DURATION)
                                                    .collect::<Vec<_>>()
                                                    .into_iter()
                                                    .rev()
                                                    .collect::<Vec<_>>();
        ui.ctx().data_mut(|id_type_map| id_type_map.insert_temp(self.manager_id, active_tooltips));
    }

    pub fn show_tooltip(&mut self, bounding_box: egui::Rect, tooltip_pos: egui::Pos2, ui: &mut egui::Ui, add_contents: impl FnOnce(&mut egui::Ui, &mut TooltipManager)) {
        let egui_id = ui.make_persistent_id(format!("Tooltip ID {}", self.tooltip_counter));
        let is_trigger_hovered = ui.interact(bounding_box, egui::Id::new(egui_id), egui::Sense::click_and_drag()).hovered();
        let is_still_open =  self.active_tooltips.len() > self.layer_counter
                                && self.active_tooltips[self.layer_counter].tooltip_id == self.tooltip_counter;

        if is_trigger_hovered {
            println!("Trigger of Tooltip Layer {} hovered", self.layer_counter);
        }
        if !is_trigger_hovered && is_still_open {
            println!("Tooltip Layer {} is still open", self.layer_counter);
        }

        if is_trigger_hovered || is_still_open {
            //Highlight the trigger to indicate it is hovered
            //  ui.painter().rect_filled(
            //     bounding_box, 
            //     0.0, 
            //     ui.visuals().weak_text_color().gamma_multiply(GAMMA_MUL_ON_HOVER)
            // );
            egui::show_tooltip_at(ui.ctx(), ui.layer_id(), egui_id, tooltip_pos, |ui| {
                let is_tooltip_hovered = ui.interact(ui.max_rect(), egui_id, egui::Sense::click_and_drag()).hovered();
                if is_tooltip_hovered {
                    println!("Tooltip Layer {} is hovered", self.layer_counter);
                }

                //If the tooltip or the trigger are hovered, set this tooltip as active for the layer
                if is_trigger_hovered || is_tooltip_hovered {
                    let active_tooltip = ActiveTooltip::new(self.tooltip_counter, Instant::now());
                    if self.layer_counter >= self.active_tooltips.len() {
                        self.active_tooltips.push(active_tooltip);
                    } else {
                        self.active_tooltips[self.layer_counter] = active_tooltip;
                    }
                }
                //Draw content on next tooltip layer
                self.layer_counter += 1;
                add_contents(ui, self);
                self.layer_counter -= 1;
            });
            ui.ctx().request_repaint_after(Duration::from_secs_f32(TOOLTIP_DURATION));
        }
        self.tooltip_counter += 1;
    }

}