//! Contains a widget to display a grid with maximizable cells.

use std::cell::RefCell;
use std::rc::Rc;

use eframe::egui;
use egui::{Ui, Layout, Align, Rect, Vec2};

/// State object held by the application, storing the currently maximized cell.
#[derive(Default, Clone)]
pub struct MaxiGridState {
    maximized: Rc<RefCell<Option<(usize, usize)>>>,
}

impl MaxiGridState {
    fn maximize(&self, cell: (usize, usize)) {
        self.maximized.borrow_mut().replace(cell);
    }

    fn minimize(&self) {
        *self.maximized.borrow_mut() = None;
    }

    fn maximized(&self) -> Option<(usize, usize)> {
        *self.maximized.borrow()
    }
}

/// Grid widget. Created and destroyed during draw.
pub struct MaxiGrid<'a> {
    cells: (usize, usize),
    available_rect: Rect,
    ui: &'a mut Ui,
    state: MaxiGridState,
    current_cell: (usize, usize),
}

impl<'a> MaxiGrid<'a> {
    pub fn new(cells: (usize, usize), ui: &'a mut Ui, state: MaxiGridState) -> Self {
        let available_rect = ui.available_rect_before_wrap();

        Self {
            cells,
            available_rect,
            ui,
            state,
            current_cell: (0, 0),
        }
    }

    fn draw_minimized(&mut self, title: &'static str, cb: impl FnOnce(&mut Ui)) {
        let top_left = self.available_rect.left_top();
        let cell_size = self.available_rect.size() / Vec2::new(self.cells.0 as f32, self.cells.1 as f32);

        let cell_top_left = top_left + (cell_size * Vec2::new(self.current_cell.0 as f32, self.current_cell.1 as f32));
        let cell_bottom_right = cell_top_left + cell_size;
        let rect = Rect::from_min_max(cell_top_left, cell_bottom_right)
            .shrink2(self.ui.style().spacing.item_spacing / 2.0);

        self.ui.put(rect, |ui: &mut Ui| {
            ui.vertical(|ui| {
                ui.add_space(3.0);
                ui.horizontal(|ui| {
                    ui.heading(title);
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        if ui.button("🗖").clicked() {
                            self.state.maximize(self.current_cell);
                        }
                    });
                });

                (cb)(ui);
            }).response
        });
    }

    fn draw_maximized(&mut self, title: &'static str, cb: impl FnOnce(&mut Ui)) {
        self.ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.heading(title);
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui.button("🗕").clicked() {
                        self.state.minimize();
                    }
                });
            });

            (cb)(ui);
        });
    }

    pub fn cell(mut self, title: &'static str, cb: impl FnOnce(&mut Ui)) -> Self {
        let maximized = self.state.maximized();
        if maximized.map(|c| c == self.current_cell).unwrap_or(false) {
            self.draw_maximized(title, cb);
        } else if maximized.is_none() {
            self.draw_minimized(title, cb);
        }

        self.current_cell.0 += 1;
        if self.current_cell.0 >= self.cells.0 {
            self.current_cell.0 = 0;
            self.current_cell.1 += 1;
        }

        self
    }
}
