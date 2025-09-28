use std::{
    marker::PhantomData,
    time::{Duration, Instant},
};

use egui::{Color32, Id, Pos2, Rect, Response, Sense, Ui, Vec2};

use crate::{
    backend::Backend,
    frontend::Frontend,
    system_diagram_components::core::constants::{GAMMA_MUL_ON_HOVER, GAMMA_MUL_ON_SELECTED, TOOLTIP_DURATION},
};

#[derive(Clone, Copy)]
struct ActivePopup {
    id: egui::Id,
    close_condition: PopupCloseCondition,
}

impl ActivePopup {
    fn new<T: PopupKind>(trigger_id: &egui::Id) -> Self {
        Self {
            id: T::id(trigger_id),
            close_condition: T::close_condition(),
        }
    }
}

#[derive(Eq, Clone, Copy)]
pub enum PopupCloseCondition {
    Timer(/*start_timer*/ Instant),
    ClickAnywhere,
}

impl PopupCloseCondition {
    pub fn is_triggered(&self) -> bool {
        return match self {
            PopupCloseCondition::Timer(instant) => instant.elapsed().as_secs_f32() > TOOLTIP_DURATION,
            PopupCloseCondition::ClickAnywhere => false,
        };
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
        };
    }
}

pub trait NotInPopupList<L: PopupList> {}
impl<K, Head, Tail> NotInPopupList<(Head, Tail)> for K
where
    K: PopupKind + Compatible<Head::Kind> + NotInPopupList<Tail>,
    Head: DisplayablePopup,
    Tail: PopupList,
{
}
impl<K: PopupKind> NotInPopupList<()> for K {}

//TODO Hans: Find a better way of doing this
pub trait Compatible<K: PopupKind> {}
impl Compatible<Tooltip> for ContextMenu {}
impl Compatible<Tooltip> for DropdownMenu {}
impl Compatible<Tooltip> for ModalDialog {}
impl Compatible<ContextMenu> for Tooltip {}
impl Compatible<ContextMenu> for DropdownMenu {}
impl Compatible<ContextMenu> for ModalDialog {}
impl Compatible<DropdownMenu> for Tooltip {}
impl Compatible<DropdownMenu> for ContextMenu {}
//impl Compatible<DropdownMenu> for ModalDialog {} => Both activated using left click, so not compatible
impl Compatible<ModalDialog> for Tooltip {}
impl Compatible<ModalDialog> for ContextMenu {}
//impl Compatible<ModalDialog> for DropdownMenu {} => Both activated using left click, so not compatible

pub trait PopupList: Sized {
    fn show_if_active(
        self,
        response: Response,
        trigger_id: Id,
        ui: &mut Ui,
        frontend: &mut Frontend,
        backend: &mut Backend,
    );
    fn add<P: DisplayablePopup>(self, popup: P) -> (P, Self) {
        return (popup, self);
    }
}

impl PopupList for () {
    fn show_if_active(
        self,
        _response: Response,
        _trigger_id: Id,
        _ui: &mut Ui,
        _frontend: &mut Frontend,
        _backend: &mut Backend,
    ) {
    }
}

impl<H, T> PopupList for (H, T)
where
    H: DisplayablePopup,
    T: PopupList,
{
    fn show_if_active(
        self,
        response: Response,
        trigger_id: Id,
        ui: &mut Ui,
        frontend: &mut Frontend,
        backend: &mut Backend,
    ) {
        NewPopupTrigger::new(self.0, &response, &trigger_id).show_if_active(ui, frontend, backend);
        self.1.show_if_active(response, trigger_id, ui, frontend, backend);
    }
}

pub struct NewPopupTrigger<'a, P>
where
    P: DisplayablePopup,
{
    popup: P,
    response: &'a Response,
    id: &'a Id,
}

impl<'a, P> NewPopupTrigger<'a, P>
where
    P: DisplayablePopup,
{
    pub fn new(popup: P, response: &'a Response, id: &'a Id) -> Self {
        Self { popup, response, id }
    }

    fn show_if_active(self, ui: &mut Ui, frontend: &mut Frontend, backend: &mut Backend) {
        return P::Kind::show_if_active(self, ui, frontend, backend);
    }
}

pub struct TriggerBuilder<L: PopupList> {
    id: Id,
    bounding_box: Rect,
    popups: L,
}

impl TriggerBuilder<()> {
    pub fn new(id: Id, bounding_box: egui::Rect) -> Self {
        return Self {
            id,
            bounding_box,
            popups: Default::default(),
        };
    }

    pub fn add_all<Contents: PopupContentList>(
        self,
        popups: Contents,
        position: Pos2,
    ) -> TriggerBuilder<Contents::AsPopupList> {
        return TriggerBuilder {
            id: self.id,
            bounding_box: self.bounding_box,
            popups: popups.position(position),
        };
    }
}

impl<L: PopupList> TriggerBuilder<L> {
    pub fn add<C /*, K, F*/>(self, position: Pos2, contents: C) -> TriggerBuilder<(PositionedPopup<C>, L)>
    where
        //K: PopupKind + NotInPopupList<L>,
        //F: FnOnce(&mut egui::Ui, &mut Frontend, &mut Backend) -> <K as PopupKind>::Response,
        C: PopupContents, //<Kind = K, Contents = F>,
    {
        return TriggerBuilder {
            id: self.id,
            bounding_box: self.bounding_box,
            popups: self.popups.add(PositionedPopup::new(position, contents)),
        };
    }

    pub fn show_active(self, ui: &mut Ui, frontend: &mut Frontend, backend: &mut Backend) {
        // Allocate Response
        let response = ui.interact(self.bounding_box, self.id, egui::Sense::click_and_drag());
        // Highlight the trigger on hover
        if response.hovered() {
            ui.painter().rect_filled(
                response.interact_rect,
                0.0,
                ui.visuals().weak_text_color().gamma_multiply(GAMMA_MUL_ON_HOVER),
            );
        }
        // Show active popups
        self.popups.show_if_active(response, self.id, ui, frontend, backend);
    }
}

pub trait PopupContentList: Sized + Clone {
    type AsPopupList: PopupList;
    fn add<C: PopupContents>(self, contents: C) -> (Self, C) {
        return (self, contents);
    }

    fn position(self, position: Pos2) -> Self::AsPopupList;
}
impl PopupContentList for () {
    type AsPopupList = ();
    fn position(self, _position: Pos2) -> Self::AsPopupList {}
}
impl<H, T> PopupContentList for (H, T)
where
    H: PopupContents,
    T: PopupContentList,
{
    type AsPopupList = (PositionedPopup<H>, T::AsPopupList);
    fn position(self, position: Pos2) -> Self::AsPopupList {
        return (PositionedPopup::new(position, self.0), self.1.position(position));
    }
}

pub trait PopupContents: Clone {
    type Kind: PopupKind;
    type Contents: FnOnce(&mut egui::Ui, &mut Frontend, &mut Backend) -> <Self::Kind as PopupKind>::Response;
    fn contents(self) -> Self::Contents;
}

impl<K, F> PopupContents for PopupContentData<K, F>
where
    K: PopupKind,
    F: FnOnce(&mut egui::Ui, &mut Frontend, &mut Backend) -> <K as PopupKind>::Response + Clone,
{
    type Kind = K;
    type Contents = F;

    fn contents(self) -> Self::Contents {
        return self.add_contents;
    }
}

#[derive(Clone)]
pub struct PopupContentData<K, F>
where
    K: PopupKind,
    F: FnOnce(&mut egui::Ui, &mut Frontend, &mut Backend) -> <K as PopupKind>::Response,
{
    popup_kind: PhantomData<K>,
    add_contents: F,
}

impl<K, F> PopupContentData<K, F>
where
    K: PopupKind,
    F: FnOnce(&mut egui::Ui, &mut Frontend, &mut Backend) -> <K as PopupKind>::Response,
{
    pub fn new(add_contents: F) -> Self {
        Self {
            add_contents,
            popup_kind: PhantomData,
        }
    }
}

pub struct PositionedPopup<Contents: PopupContents> {
    position: Pos2,
    contents: Contents,
}
impl<Contents: PopupContents> PositionedPopup<Contents> {
    pub fn new(position: Pos2, contents: Contents) -> Self {
        Self { position, contents }
    }
}

pub trait PopupResponse: Sized {
    fn union(self, other: egui::Response) -> Self;
}
impl PopupResponse for egui::Response {
    fn union(self, other: egui::Response) -> Self {
        return Self::union(&self, other);
    }
}
impl PopupResponse for egui::InnerResponse<bool> {
    fn union(self, other: egui::Response) -> Self {
        return Self::new(self.inner, self.response.union(other));
    }
}

trait FlattenPopupResponse<R: PopupResponse> {
    fn flatten(self) -> R;
}

impl<R: PopupResponse> FlattenPopupResponse<R> for egui::InnerResponse<R> {
    fn flatten(self) -> R {
        return R::union(self.inner, self.response);
    }
}

pub trait DisplayablePopup {
    type Kind: PopupKind;
    fn position(&self) -> &Pos2;
    fn show(self, ui: &mut Ui, frontend: &mut Frontend, backend: &mut Backend) -> <Self::Kind as PopupKind>::Response;
}
impl<Contents: PopupContents> DisplayablePopup for PositionedPopup<Contents> {
    type Kind = Contents::Kind;

    fn position(&self) -> &Pos2 {
        return &self.position;
    }

    fn show(self, ui: &mut Ui, frontend: &mut Frontend, backend: &mut Backend) -> <Self::Kind as PopupKind>::Response {
        return (self.contents.contents())(ui, frontend, backend);
    }
}

pub trait PopupKind: Sized + Clone {
    type Response: PopupResponse;
    fn id(trigger_id: &egui::Id) -> egui::Id;
    fn close_condition() -> PopupCloseCondition;
    fn show_if_active<'a, P>(
        trigger: NewPopupTrigger<'a, P>,
        ui: &mut Ui,
        frontend: &mut Frontend,
        backend: &mut Backend,
    ) where
        P: DisplayablePopup<Kind = Self>;
}

#[derive(Default, Clone)]
pub struct Tooltip {}
impl PopupKind for Tooltip {
    type Response = egui::Response;

    fn id(trigger_id: &egui::Id) -> egui::Id {
        return trigger_id.with("Tooltip");
    }

    fn close_condition() -> PopupCloseCondition {
        return PopupCloseCondition::Timer(Instant::now());
    }

    fn show_if_active<'a, P>(
        trigger: NewPopupTrigger<'a, P>,
        ui: &mut Ui,
        frontend: &mut Frontend,
        backend: &mut Backend,
    ) where
        P: DisplayablePopup<Kind = Self>,
    {
        PopupManager::add_tooltip(trigger, ui, frontend, backend);
    }
}

#[derive(Clone)]
pub struct ContextMenu {}
impl PopupKind for ContextMenu {
    type Response = egui::Response;

    fn id(trigger_id: &egui::Id) -> egui::Id {
        return trigger_id.with("ContextMenu");
    }

    fn close_condition() -> PopupCloseCondition {
        return PopupCloseCondition::ClickAnywhere;
    }

    fn show_if_active<'a, P>(
        trigger: NewPopupTrigger<'a, P>,
        ui: &mut Ui,
        frontend: &mut Frontend,
        backend: &mut Backend,
    ) where
        P: DisplayablePopup<Kind = Self>,
    {
        PopupManager::add_context_menu(trigger, ui, frontend, backend);
    }
}

#[derive(Clone)]
pub struct DropdownMenu {}
impl PopupKind for DropdownMenu {
    type Response = egui::Response;

    fn id(trigger_id: &egui::Id) -> egui::Id {
        return trigger_id.with("DropdownMenu");
    }

    fn close_condition() -> PopupCloseCondition {
        return PopupCloseCondition::ClickAnywhere;
    }

    fn show_if_active<'a, P>(
        trigger: NewPopupTrigger<'a, P>,
        ui: &mut Ui,
        frontend: &mut Frontend,
        backend: &mut Backend,
    ) where
        P: DisplayablePopup<Kind = Self>,
    {
        PopupManager::add_dropdown_menu(trigger, ui, frontend, backend);
    }
}

#[derive(Clone)]
pub struct ModalDialog {}
impl PopupKind for ModalDialog {
    type Response = egui::InnerResponse<bool>;

    fn id(trigger_id: &egui::Id) -> egui::Id {
        return trigger_id.with("ModalDialog");
    }

    fn close_condition() -> PopupCloseCondition {
        return PopupCloseCondition::ClickAnywhere;
    }

    fn show_if_active<'a, P>(
        trigger: NewPopupTrigger<'a, P>,
        ui: &mut Ui,
        frontend: &mut Frontend,
        backend: &mut Backend,
    ) where
        P: DisplayablePopup<Kind = Self>,
    {
        PopupManager::add_modal_dialog(trigger, ui, frontend, backend);
    }
}

pub struct PopupManager {
    active_popups: Vec<ActivePopup>,
    current_layer: usize,
    active_frame: egui::Frame,
}

impl Default for PopupManager {
    fn default() -> Self {
        Self {
            active_popups: Default::default(),
            current_layer: Default::default(),
            active_frame: Default::default(),
        }
    }
}

impl PopupManager {
    pub fn update(&mut self) {
        /*
           Set the last_hover of each active tooltip to the maximum of itself and its childeren to ensure parents stay open when children are hovered
           Afterwards, remove all tooltips with expired timers from the active tooltips list
           Apperantly, it is not possible to reverse the same iterator twice which is why the iterator is collected in the middle of the pipeline
        */
        self.active_popups = self
            .active_popups
            .iter()
            .rev()
            .scan(None, |max, cur| {
                *max = Some(PopupCloseCondition::max(max.unwrap_or(cur.close_condition), cur.close_condition));
                Some(ActivePopup {
                    id: cur.id,
                    close_condition: max.unwrap_or(cur.close_condition),
                })
            })
            .filter(|tooltip| !tooltip.close_condition.is_triggered())
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
            .collect::<Vec<_>>();
    }

    pub fn has_open_popup<T: PopupKind>(&self, trigger_id: &egui::Id) -> bool {
        return self.is_popup_open_in_current_layer(&T::id(trigger_id));
    }

    pub fn has_any_open_popup(&self, trigger_id: &egui::Id) -> bool {
        return self.is_popup_open_in_current_layer(&Tooltip::id(trigger_id))
            || self.is_popup_open_in_current_layer(&ContextMenu::id(trigger_id))
            || self.is_popup_open_in_current_layer(&DropdownMenu::id(trigger_id));
    }

    /// Add a tooltip which is displayed on hover
    fn add_tooltip<'a, P>(
        trigger: NewPopupTrigger<'a, P>,
        ui: &mut egui::Ui,
        frontend: &mut Frontend,
        backend: &mut Backend,
    ) where
        P: DisplayablePopup<Kind = Tooltip>,
    {
        let tooltip = ActivePopup::new::<Tooltip>(&trigger.id);
        let is_trigger_hovered = trigger.response.hovered();
        let is_tooltip_still_open = frontend.popup_manager.is_popup_open_in_current_layer(&tooltip.id);
        let active_close_condition =
            frontend.popup_manager.get_active_in_current_layer().map(|popup| popup.close_condition);

        //Draw tooltip on next popup layer if its close condition has a higher priority than the active tooltip
        if is_trigger_hovered && active_close_condition.map_or(true, |cond| cond < tooltip.close_condition)
            || is_tooltip_still_open
        {
            let tooltip_response = Self::show_popup(trigger.popup, P::Kind::id(&trigger.id), ui, frontend, backend);
            if is_trigger_hovered || tooltip_response.hovered() {
                frontend.popup_manager.activate_in_current_layer(tooltip);
            } else if active_close_condition.map_or(false, |cond| cond == PopupCloseCondition::ClickAnywhere)
                && frontend.popup_manager.is_current_layer_final()
            {
                frontend.popup_manager.deactivate_current_layer();
            }
            // Request a repaint to ensure the UI is updated when the tooltip closes
            ui.ctx().request_repaint_after(Duration::from_secs_f32(TOOLTIP_DURATION));
        }
    }

    /// Add a context menu which can be opened via right click. It then remains open until the user clicks elsewhere
    fn add_context_menu<'a, P>(
        trigger: NewPopupTrigger<'a, P>,
        ui: &mut egui::Ui,
        frontend: &mut Frontend,
        backend: &mut Backend,
    ) where
        P: DisplayablePopup<Kind = ContextMenu>,
    {
        let context_menu = ActivePopup::new::<ContextMenu>(&trigger.id);
        let is_trigger_right_clicked = trigger.response.secondary_clicked();
        let is_context_menu_still_open = frontend.popup_manager.is_popup_open_in_current_layer(&context_menu.id);

        //If the context menu should be drawn
        if is_trigger_right_clicked || is_context_menu_still_open {
            // Highlight the trigger to indicate it is clicked
            ui.painter().rect_filled(
                trigger.response.rect,
                0.0,
                ui.visuals().weak_text_color().gamma_multiply(GAMMA_MUL_ON_SELECTED),
            );
            //Draw context menu on next popup layer
            let context_menu_response =
                Self::show_popup(trigger.popup, P::Kind::id(&trigger.id), ui, frontend, backend);
            //Perform (de)activations
            if is_trigger_right_clicked {
                frontend.popup_manager.activate_in_current_layer(context_menu);
            } else if trigger.response.clicked_elsewhere() && context_menu_response.clicked_elsewhere() {
                frontend.popup_manager.deactivate_current_layer();
            }
        }
    }

    /// Add a dropdown menu which can be opened via left click. It then remains open until the user clicks elsewhere
    fn add_dropdown_menu<'a, P>(
        trigger: NewPopupTrigger<'a, P>,
        ui: &mut egui::Ui,
        frontend: &mut Frontend,
        backend: &mut Backend,
    ) where
        P: DisplayablePopup<Kind = DropdownMenu>,
    {
        let dropdown_menu = ActivePopup::new::<DropdownMenu>(&trigger.id);
        let is_trigger_left_clicked = trigger.response.clicked();
        let is_dropdown_menu_still_open = frontend.popup_manager.is_popup_open_in_current_layer(&dropdown_menu.id);

        //If the dropdown menu should be drawn
        if is_trigger_left_clicked || is_dropdown_menu_still_open {
            // Highlight the trigger to indicate it is clicked
            ui.painter().rect_filled(
                trigger.response.rect,
                0.0,
                ui.visuals().weak_text_color().gamma_multiply(GAMMA_MUL_ON_SELECTED),
            );
            //Draw context menu on next popup layer
            let dropdown_menu_response =
                Self::show_popup(trigger.popup, P::Kind::id(&trigger.id), ui, frontend, backend);
            //Perform (de)activations
            if is_trigger_left_clicked && !is_dropdown_menu_still_open {
                frontend.popup_manager.activate_in_current_layer(dropdown_menu);
            } else if dropdown_menu_response.clicked_elsewhere() {
                frontend.popup_manager.deactivate_current_layer();
            }
        }
    }

    /// Add a modal dialog which can be opened via left click. It then remains open until the user clicks elsewhere
    fn add_modal_dialog<'a, P>(
        trigger: NewPopupTrigger<'a, P>,
        ui: &mut egui::Ui,
        frontend: &mut Frontend,
        backend: &mut Backend,
    ) where
        P: DisplayablePopup<Kind = ModalDialog>,
    {
        let model_dialog = ActivePopup::new::<ModalDialog>(&trigger.id);
        let is_trigger_left_clicked = trigger.response.clicked();
        let is_modal_dialog_still_open = frontend.popup_manager.is_popup_open_in_current_layer(&model_dialog.id);

        //If the dropdown menu should be drawn
        if is_trigger_left_clicked || is_modal_dialog_still_open {
            // Highlight the trigger to indicate it is clicked
            ui.painter().rect_filled(
                trigger.response.rect,
                0.0,
                ui.visuals().weak_text_color().gamma_multiply(GAMMA_MUL_ON_SELECTED),
            );
            //Dim the background and make it non-interactable
            ui.ctx().memory_mut(|mem| {
                mem.set_modal_layer(egui::LayerId::new(egui::Order::Foreground, P::Kind::id(&trigger.id)))
            });

            egui::Area::new(trigger.id.with("GrayOut"))
                .sense(Sense::click_and_drag())
                .interactable(true)
                .fixed_pos(Pos2::new(0f32, 0f32))
                .order(egui::Order::Foreground)
                .show(ui.ctx(), |ui| {
                    ui.allocate_rect(ui.ctx().screen_rect(), Sense::all());
                    ui.painter().rect_filled(
                        ui.ctx().screen_rect(),
                        0.0,
                        Color32::from_rgba_unmultiplied(50, 50, 50, 150),
                    );
                });

            //Draw context menu on next popup layer
            let modal_dialog_response =
                Self::show_popup(trigger.popup, P::Kind::id(&trigger.id), ui, frontend, backend);

            //Perform (de)activations
            if is_trigger_left_clicked && !is_modal_dialog_still_open {
                frontend.popup_manager.activate_in_current_layer(model_dialog);
            } else if modal_dialog_response.inner == true {
                frontend.popup_manager.deactivate_current_layer();
            }
        }
    }

    fn is_popup_open_in_current_layer(&self, popup_id: &egui::Id) -> bool {
        return self.active_popups.len() > self.current_layer && self.active_popups[self.current_layer].id == *popup_id;
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
    fn show_popup<P: DisplayablePopup>(
        popup: P,
        id: Id,
        ui: &mut Ui,
        frontend: &mut Frontend,
        backend: &mut Backend,
    ) -> <P::Kind as PopupKind>::Response {
        frontend.popup_manager.current_layer += 1;
        frontend.popup_manager.active_frame = egui::Frame::popup(ui.style());
        let popup_response = egui::Area::new(id)
            .sense(Sense::click_and_drag())
            .interactable(true)
            .fixed_pos(*popup.position() - Vec2::new(0.0, frontend.popup_manager.active_frame.inner_margin.topf()))
            .order(egui::Order::Foreground)
            .show(ui.ctx(), |ui| {
                return frontend
                    .popup_manager
                    .active_frame
                    .show(ui, |ui| {
                        return popup.show(ui, frontend, backend);
                    })
                    .flatten();
            })
            .flatten();
        frontend.popup_manager.current_layer -= 1;
        return popup_response;
    }

    pub fn get_active_frame_bounds(&self, content_rect: egui::Rect) -> egui::Rect {
        return self.active_frame.widget_rect(content_rect);
    }
}
