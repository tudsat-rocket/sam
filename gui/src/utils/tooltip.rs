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

pub struct PopupTrigger{
    id: egui::Id,
    response: egui::Response,
}

impl PopupTrigger {

    pub fn new(ui: &mut::egui::Ui, id: egui::Id, bounding_box: egui::Rect) -> Self {
        //Create Response
        let response = ui.interact(bounding_box, egui::Id::new(format!("{} interaction", id.short_debug_format())), egui::Sense::click_and_drag());
        // Highlight the trigger on hover
        if response.hovered() {
            ui.painter().rect_filled(
                bounding_box, 
                0.0, 
                ui.visuals().weak_text_color().gamma_multiply(GAMMA_MUL_ON_HOVER)
            );
        }
        return Self{id, response};
    }

}

pub struct PopupManager {
    manager_id: egui::Id,
    active_popups: Vec<ActivePopup>,
    current_layer: usize,
}

impl PopupManager {

    pub fn new(ui: &mut egui::Ui) -> Self{
        let id = ui.make_persistent_id("Tooltip Manager");
        Self { 
            manager_id: id,
            active_popups: ui.ctx().data_mut(|id_type_map| id_type_map.get_temp::<Vec<ActivePopup>>(id)).unwrap_or(vec![]),
            current_layer: 0,
        }
    }

    pub fn save_state(self, ui: &mut egui::Ui) {
        /*
            Set the last_hover of each active tooltip to the maximum of itself and its childeren to ensure parents stay open when children are hovered
            Afterwards, remove all tooltips with expired timers from the active tooltips list
            Apperantly, it is not possible to reverse the same iterator twice which is why the iterator is collected in the middle of the pipeline
         */
        let active_tooltips = self.active_popups
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

    /// Add a tooltip which is displayed on hover
    pub fn add_tooltip(&mut self, trigger: &PopupTrigger, pos: egui::Pos2, ui: &mut egui::Ui, add_contents: impl FnOnce(&mut egui::Ui, &mut PopupManager) -> egui::Response) {
        
        let tooltip = ActivePopup::new(ui.make_persistent_id(format!("{} hover", trigger.id.short_debug_format())), PopupCloseCondition::Timer(Instant::now()));
        let is_trigger_hovered = trigger.response.hovered();
        let is_tooltip_still_open = self.is_popup_open_in_current_layer(&tooltip);
        let active_close_condition = self.get_active_in_current_layer().map(|popup| popup.close_conditon);

        //Draw tooltip on next popup layer if its close condition has a higher priority than the active tooltip
        if is_trigger_hovered && active_close_condition.map_or(true, |cond| cond < tooltip.close_conditon) || is_tooltip_still_open  {
            let tooltip_response = self.show_popup(tooltip, pos, ui, add_contents);
            if is_trigger_hovered || tooltip_response.hovered() {
                self.activate_in_current_layer(tooltip);
            } else if active_close_condition.map_or(false, |cond| cond == PopupCloseCondition::ClickAnywhere) && self.is_current_layer_final() {
                self.deactivate_current_layer();
            }
            // Request a repaint to ensure the UI is updated when the tooltip closes
            ui.ctx().request_repaint_after(Duration::from_secs_f32(TOOLTIP_DURATION));
        }
    }

    /// Add a context menu which can be opened via right click. It then remains open until the user clicks elsewhere
    pub fn add_context_menu(&mut self, trigger: &PopupTrigger, pos: egui::Pos2, ui: &mut egui::Ui, add_contents: impl FnOnce(&mut egui::Ui, &mut PopupManager) -> egui::Response) {

        let context_menu = ActivePopup::new(ui.make_persistent_id(format!("{} right click", trigger.id.short_debug_format())), PopupCloseCondition::ClickAnywhere);
        let is_trigger_right_clicked = trigger.response.secondary_clicked();
        let is_context_menu_still_open = self.is_popup_open_in_current_layer(&context_menu);

        //If the context menu should be drawn
        if is_trigger_right_clicked || is_context_menu_still_open {
            // Highlight the trigger to indicate it is clicked
            ui.painter().rect_filled(
            trigger.response.rect, 
            0.0, 
            ui.visuals().weak_text_color().gamma_multiply(GAMMA_MUL_ON_SELECTED)
            );
            //Draw context menu on next popup layer
            let context_menu_response = self.show_popup(context_menu, pos, ui, add_contents);
            //Perform (de)activations
            if is_trigger_right_clicked {
                self.activate_in_current_layer(context_menu);
                //MAYBE Requires removal of higher levels
            } else if trigger.response.clicked_elsewhere() && context_menu_response.clicked_elsewhere() {
                self.deactivate_current_layer();
            }
        }
    }

    fn is_popup_open_in_current_layer(&self, popup: &ActivePopup) -> bool {
        return self.active_popups.len() > self.current_layer
            && self.active_popups[self.current_layer].id == popup.id;
    }

    fn activate_in_current_layer(&mut self, popup: ActivePopup) {
        if self.current_layer >= self.active_popups.len() {
            self.active_popups.push(popup);
        } else {
            self.active_popups[self.current_layer] = popup;
        }
    }

    /// Remove this popup and all its children
    fn deactivate_current_layer(&mut self) {
        self.active_popups.truncate(self.current_layer);
    }

    /// Return the active popup if the current layer contains an active popup
    fn get_active_in_current_layer(&self) -> Option<&ActivePopup> {
        if self.active_popups.len() > self.current_layer {
            Some(&self.active_popups[self.current_layer])
        } else {
            None
        }
    }

    /// Returns true if the current layer is the last layer with active popups
    fn is_current_layer_final(&self) -> bool {
        return self.current_layer == self.active_popups.len() - 1;
    }

    /// Show the popup in a new layer at the given positions
    fn show_popup(&mut self, popup: ActivePopup, pos: egui::Pos2, ui: &mut egui::Ui, add_contents: impl FnOnce(&mut egui::Ui, &mut PopupManager) -> egui::Response) -> egui::Response{
        let mut popup_response = ui.allocate_response(egui::Vec2::ZERO, egui::Sense::click_and_drag());

        egui::show_tooltip_at(ui.ctx(), ui.layer_id(), popup.id, pos, |ui| {    
            self.current_layer += 1;
            popup_response = add_contents(ui, self);
            self.current_layer -= 1;
        });

        return popup_response;
    }
}