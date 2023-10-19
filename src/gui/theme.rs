use eframe::egui;
use egui::{Color32, Context, Style};

#[derive(Debug)]
pub struct ThemeColors {
    pub background: Color32,
    pub background_weak: Color32,
    pub background_strong: Color32,
    pub background_very_strong: Color32,
    pub foreground: Color32,
    pub foreground_weak: Color32,
    pub foreground_very_weak: Color32,
    pub foreground_strong: Color32,
}

impl ThemeColors {
    pub fn new(ctx: &Context) -> Self {
        if ctx.style().visuals.dark_mode {
            Self::dark()
        } else {
            Self::light()
        }
    }

    pub fn dark() -> Self {
        Self {
            background: Color32::from_rgb(0x1d, 0x20, 0x21),
            background_weak: Color32::from_rgb(0x0e, 0x10, 0x11),
            background_strong: Color32::from_rgb(0x3c, 0x38, 0x36),
            background_very_strong: Color32::from_rgb(0x50, 0x49, 0x45),
            foreground: Color32::from_rgb(0xfb, 0xf1, 0xc7),
            foreground_weak: Color32::from_rgb(0xd5, 0xc4, 0xa1),
            foreground_very_weak: Color32::from_rgb(0x50, 0x49, 0x45),
            foreground_strong: Color32::from_rgb(0xff, 0xf8, 0xce),
        }
    }

    pub fn light() -> Self {
        Self {
            background: Color32::from_rgb(0xfc, 0xfa, 0xf0),
            background_weak: Color32::from_rgb(0xf3, 0xf1, 0xe7),
            background_strong: Color32::from_rgb(0xf9, 0xf5, 0xdf),
            background_very_strong: Color32::from_rgb(0xff, 0xf8, 0xce),
            foreground: Color32::from_rgb(0x3c, 0x38, 0x36),
            foreground_weak: Color32::from_rgb(0x50, 0x49, 0x45),
            foreground_very_weak: Color32::from_rgb(0xa8, 0x99, 0x84),
            foreground_strong: Color32::from_rgb(0x28, 0x28, 0x28),
        }
    }

    pub fn apply(&self, style: &mut Style) {
        style.visuals.override_text_color = Some(self.foreground);
        style.visuals.panel_fill = self.background;
        style.visuals.extreme_bg_color = self.background_weak;
        style.visuals.widgets.noninteractive.bg_fill = self.background;
        style.visuals.widgets.noninteractive.bg_stroke.color = self.background_very_strong;
        style.visuals.widgets.inactive.bg_fill = self.background_strong;
        style.visuals.widgets.hovered.bg_fill = self.background_very_strong;
        style.visuals.widgets.hovered.bg_stroke.color = self.foreground;
        style.visuals.widgets.hovered.fg_stroke.color = self.foreground;
        style.visuals.widgets.active.bg_fill = self.background_strong;
        style.visuals.widgets.active.bg_stroke.color = self.foreground_strong;
        style.visuals.widgets.active.fg_stroke.color = self.foreground_strong;
        style.visuals.widgets.open.fg_stroke.color = self.foreground_very_weak;
    }
}
