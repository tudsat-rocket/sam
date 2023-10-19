use eframe::egui;

pub trait MiscUiExt {
    fn toggle_button(&mut self, value: &mut bool, text_false: &str, text_true: &str);
}

impl MiscUiExt for egui::Ui {
    fn toggle_button(&mut self, value: &mut bool, text_false: &str, text_true: &str) {
        let text = if *value { text_true } else { text_false };

        if self.button(text).clicked() {
            *value = !(*value);
        }
    }
}
