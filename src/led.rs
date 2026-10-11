//! Aura RGB LED ring control types for the Grow R503 sensor.

use core::str::FromStr;

/// Error returned when parsing a [`LedColor`] or [`LedMode`] from a string fails.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParseLedError {
    kind: &'static str,
}

impl core::fmt::Display for ParseLedError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "invalid LED {}", self.kind)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for ParseLedError {}

/// Colors supported by the Grow R503 circular Aura LED ring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LedColor {
    Red = 0x01,
    Blue = 0x02,
    Purple = 0x03,
    Green = 0x04,
    Yellow = 0x05,
    Cyan = 0x06,
    White = 0x07,
}

impl FromStr for LedColor {
    type Err = ParseLedError;

    /// Parses a case-insensitive color name (e.g. `"red"`, `"Blue"`).
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let trimmed = s.trim();
        let table = [
            ("red", LedColor::Red),
            ("blue", LedColor::Blue),
            ("purple", LedColor::Purple),
            ("green", LedColor::Green),
            ("yellow", LedColor::Yellow),
            ("cyan", LedColor::Cyan),
            ("white", LedColor::White),
        ];
        table
            .iter()
            .find(|(name, _)| trimmed.eq_ignore_ascii_case(name))
            .map(|&(_, color)| color)
            .ok_or(ParseLedError { kind: "color" })
    }
}

/// Operating animation modes for the Aura LED ring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LedMode {
    /// Breathing / pulsing glow effect.
    Breathing = 0x01,
    /// Fast flashing animation.
    Flashing = 0x02,
    /// Constant on without animation.
    AlwaysOn = 0x03,
    /// Completely turned off.
    AlwaysOff = 0x04,
    /// Gradual fade-in animation.
    GradualOn = 0x05,
    /// Gradual fade-out animation.
    GradualOff = 0x06,
}

impl FromStr for LedMode {
    type Err = ParseLedError;

    /// Parses a case-insensitive mode name (e.g. `"breathing"`, `"on"`, `"gradual_off"`).
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let trimmed = s.trim();
        let table = [
            ("breathing", LedMode::Breathing),
            ("flashing", LedMode::Flashing),
            ("on", LedMode::AlwaysOn),
            ("always_on", LedMode::AlwaysOn),
            ("off", LedMode::AlwaysOff),
            ("always_off", LedMode::AlwaysOff),
            ("gradual_on", LedMode::GradualOn),
            ("gradual_off", LedMode::GradualOff),
        ];
        table
            .iter()
            .find(|(name, _)| trimmed.eq_ignore_ascii_case(name))
            .map(|&(_, mode)| mode)
            .ok_or(ParseLedError { kind: "mode" })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_color_and_mode() {
        assert_eq!(" Cyan ".parse::<LedColor>(), Ok(LedColor::Cyan));
        assert!("pink".parse::<LedColor>().is_err());
        assert_eq!("ALWAYS_ON".parse::<LedMode>(), Ok(LedMode::AlwaysOn));
        assert!("strobe".parse::<LedMode>().is_err());
    }
}
