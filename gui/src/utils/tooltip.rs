use std::time::{Duration, Instant};

use crate::system_diagram_components::core::constants::{GAMMA_MUL_ON_HOVER, TOOLTIP_DURATION};

#[derive(Clone, Copy)]
struct ActiveTooltip {
    tooltip_id: egui::Id,
    last_hover: Instant,
}

impl ActiveTooltip {

    fn new(tooltip_id: egui::Id,  last_hover: Instant) -> Self {
        Self { tooltip_id, last_hover }
    }

}

pub struct TooltipManager {
    manager_id: egui::Id,
    ///Specifies the tooltip ID of the active tooltip for a given layer ID
    active_tooltips: Vec<ActiveTooltip>,
    layer_counter: usize,
}

impl TooltipManager {

    pub fn new(ui: &mut egui::Ui) -> Self{
        let id = ui.make_persistent_id("Tooltip Manager");
        Self { 
            manager_id: id,
            active_tooltips: ui.ctx().data_mut(|id_type_map| id_type_map.get_temp::<Vec<ActiveTooltip>>(id)).unwrap_or(vec![]),
            layer_counter: 0,
        }
    }

    pub fn save_state(self, ui: &mut egui::Ui) {
        /*
            Set the last_hover of each active tooltip to the maximum of itself and its childeren to ensure parents stay open when children are hovered
            Afterwards, remove all tooltips with expired timers from the active tooltips list
            Apperantly, it is not possible to reverse the same iterator twice which is why the iterator is collected in the middle of the pipeline
         */
        let active_tooltips = self.active_tooltips
                                .clone()
                                .iter()
                                .rev()
                                .scan(None, |max, cur| {
                                    *max = Some(Instant::max(max.unwrap_or(cur.last_hover), cur.last_hover));
                                    Some(ActiveTooltip::new(cur.tooltip_id, Instant::max(max.unwrap_or(cur.last_hover), cur.last_hover)))
                                })
                                .filter(|tooltip| tooltip.last_hover.elapsed().as_secs_f32() < TOOLTIP_DURATION)
                                .collect::<Vec<_>>()
                                .into_iter()
                                .rev()
                                .collect::<Vec<_>>();
        ui.ctx().data_mut(|id_type_map| id_type_map.insert_temp(self.manager_id, active_tooltips));
    }

    pub fn show_tooltip(&mut self, trigger_id: egui::Id, bounding_box: egui::Rect, tooltip_pos: egui::Pos2, ui: &mut egui::Ui, add_contents: impl FnOnce(&mut egui::Ui, &mut TooltipManager) -> egui::Response) {
        let trigger_id_str = trigger_id.short_debug_format();
        let tooltip_id = ui.make_persistent_id(format!("{trigger_id_str} hovered"));
        let is_trigger_hovered = ui.interact(bounding_box, egui::Id::new(format!("{trigger_id_str} interaction")), egui::Sense::click_and_drag()).hovered();
        let is_still_open =  self.active_tooltips.len() > self.layer_counter
                                && self.active_tooltips[self.layer_counter].tooltip_id == tooltip_id;

        if is_trigger_hovered || is_still_open {
            // Highlight the trigger to indicate it is hovered
             ui.painter().rect_filled(
                bounding_box, 
                0.0, 
                ui.visuals().weak_text_color().gamma_multiply(GAMMA_MUL_ON_HOVER)
            );

            egui::show_tooltip_at(ui.ctx(), ui.layer_id(), tooltip_id, tooltip_pos, |ui| {    
                //Draw content on next tooltip layer
                self.layer_counter += 1;
                let response = add_contents(ui, self);
                self.layer_counter -= 1;

                //If the tooltip or the trigger are hovered, set this tooltip as active for the layer
                if is_trigger_hovered || response.hovered() {
                    let active_tooltip = ActiveTooltip::new(tooltip_id, Instant::now());
                    if self.layer_counter >= self.active_tooltips.len() {
                        self.active_tooltips.push(active_tooltip);
                    } else {
                        self.active_tooltips[self.layer_counter] = active_tooltip;
                    }
                }

            });

            // Request a repaint to ensure the UI is updated when the tooltip closes
            ui.ctx().request_repaint_after(Duration::from_secs_f32(TOOLTIP_DURATION));
        }
    }

}