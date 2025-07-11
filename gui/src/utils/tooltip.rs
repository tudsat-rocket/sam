use std::time::{Duration, Instant};

use crate::system_diagram_components::core::constants::{GAMMA_MUL_ON_HOVER, TOOLTIP_DURATION};

#[derive(Clone, Copy)]
struct ActiveTooltip {
    tooltip_id: egui::Id,
    close_conditon: TooltipCloseCondition,
}

impl ActiveTooltip {

    fn new(tooltip_id: egui::Id,  close_conditon: TooltipCloseCondition) -> Self {
        Self { tooltip_id, close_conditon }
    }

}

#[derive(Eq, Clone, Copy)]
enum TooltipCloseCondition {
    Timer(/*start_timer*/ Instant),
    ClickAnywhere
}

impl TooltipCloseCondition {

    pub fn is_triggered(&self) -> bool {
        return match self {
            TooltipCloseCondition::Timer(instant) => instant.elapsed().as_secs_f32() > TOOLTIP_DURATION,
            TooltipCloseCondition::ClickAnywhere => false,
        }
    }

}

impl PartialEq for TooltipCloseCondition {
    fn eq(&self, other: &Self) -> bool {
        return self.cmp(other) == std::cmp::Ordering::Equal;
    }
}

impl PartialOrd for TooltipCloseCondition {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        return Some(self.cmp(other));
    }
}

impl Ord for TooltipCloseCondition {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        return match (self, other) {
            (TooltipCloseCondition::Timer(i1), TooltipCloseCondition::Timer(i2)) => return i1.cmp(i2),
            (TooltipCloseCondition::Timer(_), TooltipCloseCondition::ClickAnywhere) => std::cmp::Ordering::Less,
            (TooltipCloseCondition::ClickAnywhere, TooltipCloseCondition::Timer(_)) => std::cmp::Ordering::Greater,
            (TooltipCloseCondition::ClickAnywhere, TooltipCloseCondition::ClickAnywhere) => std::cmp::Ordering::Equal,
        }
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
                                    *max = Some(TooltipCloseCondition::max(max.unwrap_or(cur.close_conditon), cur.close_conditon));
                                    Some(ActiveTooltip::new(cur.tooltip_id, max.unwrap_or(cur.close_conditon)))
                                })
                                .filter(|tooltip| !tooltip.close_conditon.is_triggered())
                                .collect::<Vec<_>>()
                                .into_iter()
                                .rev()
                                .collect::<Vec<_>>();
        //println!("{}", active_tooltips.len());
        ui.ctx().data_mut(|id_type_map| id_type_map.insert_temp(self.manager_id, active_tooltips));
    }

    pub fn show_tooltip(&mut self, trigger_id: egui::Id, bounding_box: egui::Rect, tooltip_pos: egui::Pos2, ui: &mut egui::Ui, add_contents: impl FnOnce(&mut egui::Ui, &mut TooltipManager) -> egui::Response) {
        let trigger_id_str = trigger_id.short_debug_format();
        let trigger_response = ui.interact(bounding_box, egui::Id::new(format!("{trigger_id_str} interaction")), egui::Sense::click_and_drag());

        let tooltip_secondary_clicked = ActiveTooltip::new(ui.make_persistent_id(format!("{trigger_id_str} secondary_clicked")), TooltipCloseCondition::ClickAnywhere);
        let is_still_open_secondary_clicked = self.active_tooltips.len() > self.layer_counter
                                                 && self.active_tooltips[self.layer_counter].tooltip_id == tooltip_secondary_clicked.tooltip_id;
        let has_priority_secondary_clicked = self.active_tooltips.len() <= self.layer_counter 
                                                || tooltip_secondary_clicked.close_conditon > self.active_tooltips[self.layer_counter].close_conditon;


        //println!("{} {} {}", trigger_response.secondary_clicked(), is_still_open_secondary_clicked, has_priority_secondary_clicked);

        if (trigger_response.secondary_clicked() || is_still_open_secondary_clicked) && has_priority_secondary_clicked {
            // Highlight the trigger to indicate it is clicked
             ui.painter().rect_filled(
                bounding_box, 
                0.0, 
                ui.visuals().weak_text_color().gamma_multiply(GAMMA_MUL_ON_HOVER)
            );
            // Set this tooltip active for the current layer
            if self.layer_counter >= self.active_tooltips.len() {
                self.active_tooltips.push(tooltip_secondary_clicked);
            } else {
                self.active_tooltips[self.layer_counter] = tooltip_secondary_clicked;
            }
            //Draw content on next tooltip layer
            egui::show_tooltip_at(ui.ctx(), ui.layer_id(), tooltip_secondary_clicked.tooltip_id, tooltip_pos, |ui| {    
                self.layer_counter += 1;
                let _ = add_contents(ui, self);
                self.layer_counter -= 1;
            });
            return;
        }

        let tooltip_hovered = ActiveTooltip::new(ui.make_persistent_id(format!("{trigger_id_str} hovered")), TooltipCloseCondition::Timer(Instant::now()));
        let is_still_open_hovered =  self.active_tooltips.len() > self.layer_counter
                                && self.active_tooltips[self.layer_counter].tooltip_id == tooltip_hovered.tooltip_id;
        let has_priority_hovered = self.active_tooltips.len() <= self.layer_counter 
                                || tooltip_hovered.close_conditon > self.active_tooltips[self.layer_counter].close_conditon;

        if (trigger_response.hovered() || is_still_open_hovered) && has_priority_hovered {
            // Highlight the trigger to indicate it is hovered
             ui.painter().rect_filled(
                bounding_box, 
                0.0, 
                ui.visuals().weak_text_color().gamma_multiply(GAMMA_MUL_ON_HOVER)
            );
            //Draw content on next tooltip layer
            egui::show_tooltip_at(ui.ctx(), ui.layer_id(), tooltip_hovered.tooltip_id, tooltip_pos, |ui| {    
                self.layer_counter += 1;
                let tooltip_response = add_contents(ui, self);
                self.layer_counter -= 1;

                // Set this tooltip active for the current layer if it or its trigger are hovered
                if self.layer_counter >= self.active_tooltips.len() {
                    self.active_tooltips.push(tooltip_hovered);
                } else  if trigger_response.hovered() || tooltip_response.hovered(){
                    self.active_tooltips[self.layer_counter] = tooltip_hovered;
                }
            });

            // Request a repaint to ensure the UI is updated when the tooltip closes
            ui.ctx().request_repaint_after(Duration::from_secs_f32(TOOLTIP_DURATION));
        }
    }

}