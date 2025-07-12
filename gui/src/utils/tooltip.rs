use std::time::{Duration, Instant};

use crate::system_diagram_components::core::constants::{GAMMA_MUL_ON_HOVER, GAMMA_MUL_ON_SELECTED, TOOLTIP_DURATION};

#[derive(Clone, Copy)]
struct ActivePopup {
    id: egui::Id,
    close_conditon: PopupCloseCondition,
}

impl ActivePopup {

    fn new(id: egui::Id,  close_conditon: PopupCloseCondition) -> Self {
        Self { id, close_conditon }
    }

}

#[derive(Eq, Clone, Copy)]
enum PopupCloseCondition {
    Timer(/*start_timer*/ Instant),
    ClickAnywhere
}

impl PopupCloseCondition {

    pub fn is_triggered(&self) -> bool {
        return match self {
            PopupCloseCondition::Timer(instant) => instant.elapsed().as_secs_f32() > TOOLTIP_DURATION,
            PopupCloseCondition::ClickAnywhere => false,
        }
    }

}

impl PartialEq for PopupCloseCondition {
    fn eq(&self, other: &Self) -> bool {
        return self.cmp(other) == std::cmp::Ordering::Equal;
    }
}

impl PartialOrd for PopupCloseCondition {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        return Some(self.cmp(other));
    }
}

impl Ord for PopupCloseCondition {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        return match (self, other) {
            (PopupCloseCondition::Timer(i1), PopupCloseCondition::Timer(i2)) => return i1.cmp(i2),
            (PopupCloseCondition::Timer(_), PopupCloseCondition::ClickAnywhere) => std::cmp::Ordering::Less,
            (PopupCloseCondition::ClickAnywhere, PopupCloseCondition::Timer(_)) => std::cmp::Ordering::Greater,
            (PopupCloseCondition::ClickAnywhere, PopupCloseCondition::ClickAnywhere) => std::cmp::Ordering::Equal,
        }
    }
}

pub struct PopupManager {
    manager_id: egui::Id,
    active_tooltips: Vec<ActivePopup>,
    layer_counter: usize,
}

impl PopupManager {

    pub fn new(ui: &mut egui::Ui) -> Self{
        let id = ui.make_persistent_id("Tooltip Manager");
        Self { 
            manager_id: id,
            active_tooltips: ui.ctx().data_mut(|id_type_map| id_type_map.get_temp::<Vec<ActivePopup>>(id)).unwrap_or(vec![]),
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
                                    *max = Some(PopupCloseCondition::max(max.unwrap_or(cur.close_conditon), cur.close_conditon));
                                     Some(ActivePopup::new(cur.id, max.unwrap_or(cur.close_conditon)))
                                })
                                .filter(|tooltip| !tooltip.close_conditon.is_triggered())
                                .collect::<Vec<_>>()
                                .into_iter()
                                .rev()
                                .collect::<Vec<_>>();
        ui.ctx().data_mut(|id_type_map| id_type_map.insert_temp(self.manager_id, active_tooltips));
    }

    pub fn show_tooltip(&mut self, trigger_id: egui::Id, bounding_box: egui::Rect, tooltip_pos: egui::Pos2, ui: &mut egui::Ui, add_contents: impl FnOnce(&mut egui::Ui, &mut PopupManager) -> egui::Response) {
        let trigger_id_str = trigger_id.short_debug_format();
        let trigger_response = ui.interact(bounding_box, egui::Id::new(format!("{trigger_id_str} interaction")), egui::Sense::click_and_drag());

        
        let context_menu = ActivePopup::new(ui.make_persistent_id(format!("{trigger_id_str} right click")), PopupCloseCondition::ClickAnywhere);
        let tooltip = ActivePopup::new(ui.make_persistent_id(format!("{trigger_id_str} hover")), PopupCloseCondition::Timer(Instant::now()));

        let is_trigger_right_clicked = trigger_response.secondary_clicked();
        let is_trigger_hovered = trigger_response.hovered();
        let is_tooltip_still_open = self.active_tooltips.len() > self.layer_counter
                                        && self.active_tooltips[self.layer_counter].id == tooltip.id;
        let is_context_menu_still_open = self.active_tooltips.len() > self.layer_counter
                                             && self.active_tooltips[self.layer_counter].id == context_menu.id;

        let active_popup_close_condition = 
            if self.active_tooltips.len() > self.layer_counter {
                Some(self.active_tooltips[self.layer_counter].close_conditon)
            } else {
                None
            };

        // Highlight the trigger on hover
        if is_trigger_hovered {
            ui.painter().rect_filled(
                bounding_box, 
                0.0, 
                ui.visuals().weak_text_color().gamma_multiply(GAMMA_MUL_ON_HOVER)
            );
        }

        if is_trigger_right_clicked {
            // Set this context menu active for the current layer
            if self.layer_counter >= self.active_tooltips.len() {
                self.active_tooltips.push(context_menu);
            } else {
                self.active_tooltips[self.layer_counter] = context_menu;
            }
            // Remove any old higher level popups
            self.active_tooltips.truncate(self.layer_counter + 1);
        }

        //If the context menu should be drawn
        if is_trigger_right_clicked || is_context_menu_still_open {
            // Highlight the trigger to indicate it is clicked
            ui.painter().rect_filled(
            bounding_box, 
            0.0, 
            ui.visuals().weak_text_color().gamma_multiply(GAMMA_MUL_ON_SELECTED)
            );
            //Draw context menu on next popup layer
            egui::show_tooltip_at(ui.ctx(), ui.layer_id(), context_menu.id, tooltip_pos, |ui| {    
                self.layer_counter += 1;
                let context_menu_response = add_contents(ui, self);
                self.layer_counter -= 1;

                //When clicking elsewhere, close this popup and its children
                if trigger_response.clicked_elsewhere() && context_menu_response.clicked_elsewhere() {
                    self.active_tooltips.truncate(self.layer_counter);
                }
            });

            return;
        }

        //Draw tooltip on next popup layer if its close condition has a higher priority than the active tooltip
        if is_trigger_hovered && active_popup_close_condition.map(|cc| cc < tooltip.close_conditon).unwrap_or(true) || is_tooltip_still_open  {
            egui::show_tooltip_at(ui.ctx(), ui.layer_id(), tooltip.id, tooltip_pos, |ui| {    
                self.layer_counter += 1;
                let tooltip_response = add_contents(ui, self);
                self.layer_counter -= 1;

                // Set this tooltip active for the current layer if it or its trigger are hovered
                // Otherwise if this tooltip should close on click and has no children, remove it
                if self.layer_counter >= self.active_tooltips.len() {
                    self.active_tooltips.push(tooltip);
                } else if is_trigger_hovered || tooltip_response.hovered(){
                    self.active_tooltips[self.layer_counter] = tooltip;
                } else if self.active_tooltips[self.layer_counter].close_conditon == PopupCloseCondition::ClickAnywhere && self.layer_counter == self.active_tooltips.len() - 1 {
                    self.active_tooltips.truncate(self.layer_counter);
                }
            });

            // Request a repaint to ensure the UI is updated when the tooltip closes
            ui.ctx().request_repaint_after(Duration::from_secs_f32(TOOLTIP_DURATION));
        }

    }

}