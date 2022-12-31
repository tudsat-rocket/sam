//! Contains a widget to display a grid with maximizable cells.

use std::cell::RefCell;
use std::rc::Rc;

use eframe::egui;
use egui::{Ui, Response, Layout, Align};

/// State object held by the application, storing the currently maximized cell.
#[derive(Default, Clone)]
pub struct MaxiGridState {
    maximized: Rc<RefCell<Option<(usize, usize)>>>,
}

impl MaxiGridState {
    fn maximize(&self, row: usize, col: usize) {
        self.maximized.borrow_mut().replace((row, col));
    }

    fn minimize(&self) {
        *self.maximized.borrow_mut() = None;
    }

    fn maximized(&self) -> Option<(usize, usize)> {
        *self.maximized.borrow()
    }
}

/// Grid widget. Created and destroyed during draw.
pub struct MaxiGrid {
    id: &'static str,
    state: MaxiGridState,
    rows: Vec<Vec<(&'static str, Box<dyn FnOnce(&mut Ui)>)>>
}

impl MaxiGrid {
    pub fn new(id: &'static str, state: MaxiGridState) -> Self {
        Self {
            id,
            state,
            rows: vec![Vec::new()]
        }
    }

    pub fn add_cell(&mut self, title: &'static str, cb: impl FnOnce(&mut Ui) + 'static) {
        self.rows.last_mut()
            .unwrap()
            .push((title, Box::new(cb)))
    }

    pub fn end_row(&mut self) {
        self.rows.push(Vec::new());
    }

    // TODO: reduce code reuse here...
    fn show_minimized(self, ui: &mut Ui) -> Response {
        let style = ui.style();
        let xspacing = style.spacing.item_spacing.x * (3.0 / 4.0);
        let yspacing = style.spacing.item_spacing.y / 1.5;

        egui::Grid::new(self.id)
            .min_col_width(ui.available_width() / 4.0 - xspacing)
            .max_col_width(ui.available_width() / 4.0 - xspacing)
            .min_row_height(ui.available_height() / 3.0 - yspacing)
            .show(ui, |ui| {
                for (i, row) in self.rows.into_iter().enumerate() {
                    if i != 0 {
                        ui.end_row();
                    }

                    for (j, (heading, widget)) in row.into_iter().enumerate() {
                        ui.vertical(|ui| {
                            ui.horizontal(|ui| {
                                ui.heading(heading);

                                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                                    if ui.button("🗖").clicked() {
                                        self.state.maximize(i, j);
                                    }
                                });
                            });

                            (widget)(ui);
                        });
                    }
                }
            }).response
    }

    // ... and here.
    fn show_maximized(self, ui: &mut Ui, r: usize, c: usize) -> Response {
        for (i, row) in self.rows.into_iter().enumerate() {
            for (j, (heading, widget)) in row.into_iter().enumerate() {
                if i == r && j == c {
                    return ui.vertical(|ui| {
                        ui.horizontal(|ui| {
                            ui.heading(heading);

                            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                                if ui.button("🗕").clicked() {
                                    self.state.minimize();
                                }
                            });
                        });

                        (widget)(ui);
                    }).response
                }
            }
        }

        unreachable!() // TODO
    }
}

impl egui::Widget for MaxiGrid {
    fn ui(self, ui: &mut Ui) -> Response {
        if let Some((r, c)) = self.state.maximized() {
            self.show_maximized(ui, r, c)
        } else {
            self.show_minimized(ui)
        }
    }
}
