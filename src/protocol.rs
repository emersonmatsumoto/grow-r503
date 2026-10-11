//! Low-level ZhianTec / Syno packet framing and protocol definitions.

use core::ops::Range;

pub const START_CODE: [u8; 2] = [0xEF, 0x01];
pub const DEFAULT_ADDRESS: [u8; 4] = [0xFF, 0xFF, 0xFF, 0xFF];

pub const PID_COMMAND: u8 = 0x01;
pub const PID_DATA: u8 = 0x02;
pub const PID_ACK: u8 = 0x07;
pub const PID_END: u8 = 0x08;

// Primary instruction opcodes
pub const CMD_GET_IMAGE: u8 = 0x01;
pub const CMD_IMAGE_TO_TEMPLATE: u8 = 0x02;
pub const CMD_MATCH: u8 = 0x03;
pub const CMD_SEARCH: u8 = 0x04;
pub const CMD_REG_MODEL: u8 = 0x05;
pub const CMD_STORE: u8 = 0x06;
pub const CMD_LOAD_TEMPLATE: u8 = 0x07;
pub const CMD_UPLOAD_TEMPLATE: u8 = 0x08;
pub const CMD_DOWNLOAD_TEMPLATE: u8 = 0x09;
pub const CMD_UPLOAD_IMAGE: u8 = 0x0A;
pub const CMD_DOWNLOAD_IMAGE: u8 = 0x0B;
pub const CMD_DELETE_TEMPLATE: u8 = 0x0C;
pub const CMD_EMPTY_DATABASE: u8 = 0x0D;
pub const CMD_WRITE_REG: u8 = 0x0E;
pub const CMD_READ_SYS_PARAMS: u8 = 0x0F;
pub const CMD_SET_PASSWORD: u8 = 0x12;
pub const CMD_VERIFY_PASSWORD: u8 = 0x13;
pub const CMD_GET_RANDOM_CODE: u8 = 0x14;
pub const CMD_SET_ADDRESS: u8 = 0x15;
pub const CMD_READ_INF_PAGE: u8 = 0x16;
pub const CMD_WRITE_NOTEPAD: u8 = 0x18;
pub const CMD_READ_NOTEPAD: u8 = 0x19;
pub const CMD_TEMPLATE_COUNT: u8 = 0x1D;
pub const CMD_READ_INDEX_TABLE: u8 = 0x1F;
pub const CMD_AUTO_ENROLL: u8 = 0x31;
pub const CMD_AUTO_IDENTIFY: u8 = 0x32;
pub const CMD_AURA_LED_CONFIG: u8 = 0x35;
pub const CMD_READ_PRODUCT_INFO: u8 = 0x3C;

/// Sensor response confirmation codes returned in acknowledge packets.
///
/// This enum is `#[non_exhaustive]`: new codes may be added in minor releases.
/// Codes not recognized by this crate are preserved in [`ConfirmCode::Unknown`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum ConfirmCode {
    /// `0x00`: Command executed successfully.
    Ok,
    /// `0x01`: Error when receiving the data package.
    PacketReceiveError,
    /// `0x02`: No finger on the sensor.
    NoFinger,
    /// `0x03`: Failed to enroll the finger (image capture failed).
    EnrollImageFail,
    /// `0x04`: Image too dry / too light to generate a character file.
    ImageTooDry,
    /// `0x05`: Image too wet / too blurry to generate a character file.
    ImageTooWet,
    /// `0x06`: Image too disorderly to generate a character file.
    ImageMessy,
    /// `0x07`: Too few feature points / image too small.
    FeatureTooFew,
    /// `0x08`: Fingerprints do not match.
    FingerNotMatch,
    /// `0x09`: No matching fingerprint found in the library.
    NotFound,
    /// `0x0A`: Failed to combine the character files.
    MergeFail,
    /// `0x0B`: Page ID is beyond the fingerprint library range.
    InvalidPageId,
    /// `0x0C`: Error reading the template from the library, or template invalid.
    ReadModelFail,
    /// `0x0D`: Error uploading the template.
    UploadTemplateFail,
    /// `0x0E`: Module cannot receive the following data packages.
    DataReceiveFail,
    /// `0x0F`: Error uploading the image.
    UploadImageFail,
    /// `0x10`: Failed to delete the template.
    DeleteFail,
    /// `0x11`: Failed to clear the fingerprint library.
    ClearFail,
    /// `0x13`: Wrong password.
    WrongPassword,
    /// `0x15`: Failed to generate the image due to lack of a valid primary image.
    InvalidImage,
    /// `0x17`: Residual fingerprint, or finger did not move between captures.
    ResidualFingerprint,
    /// `0x18`: Error writing to flash.
    FlashWriteError,
    /// `0x19`: Undefined error.
    UndefinedError,
    /// `0x1A`: Invalid register number.
    InvalidRegister,
    /// `0x1B`: Incorrect register configuration.
    RegisterConfigError,
    /// `0x1C`: Wrong notepad page number.
    InvalidNotepadPage,
    /// `0x1D`: Failed to operate the communication port.
    PortOperationFail,
    /// `0x1E`: Automatic enrollment failed.
    AutoEnrollFail,
    /// `0x1F`: Fingerprint library is full.
    LibraryFull,
    /// `0x20`: Incorrect device address.
    InvalidAddress,
    /// `0x21`: Password must be verified first.
    PasswordRequired,
    /// `0x22`: Fingerprint template slot is not empty.
    TemplateNotEmpty,
    /// `0x24`: Fingerprint library is empty.
    LibraryEmpty,
    /// `0x26`: Operation timed out on the module.
    DeviceTimeout,
    /// `0x27`: Fingerprint already exists in the library.
    AlreadyExists,
    /// `0x29`: Sensor hardware error.
    SensorHardwareError,
    /// `0xFC`: Unsupported command.
    UnsupportedCommand,
    /// `0xFD`: Hardware error.
    HardwareError,
    /// `0xFE`: Command execution failure.
    CommandFailed,
    /// Any confirmation byte not recognized by this crate (raw value preserved).
    Unknown(u8),
}

impl ConfirmCode {
    /// Converts a raw confirmation byte from an acknowledge packet into a [`ConfirmCode`].
    pub fn from_u8(code: u8) -> Self {
        match code {
            0x00 => ConfirmCode::Ok,
            0x01 => ConfirmCode::PacketReceiveError,
            0x02 => ConfirmCode::NoFinger,
            0x03 => ConfirmCode::EnrollImageFail,
            0x04 => ConfirmCode::ImageTooDry,
            0x05 => ConfirmCode::ImageTooWet,
            0x06 => ConfirmCode::ImageMessy,
            0x07 => ConfirmCode::FeatureTooFew,
            0x08 => ConfirmCode::FingerNotMatch,
            0x09 => ConfirmCode::NotFound,
            0x0A => ConfirmCode::MergeFail,
            0x0B => ConfirmCode::InvalidPageId,
            0x0C => ConfirmCode::ReadModelFail,
            0x0D => ConfirmCode::UploadTemplateFail,
            0x0E => ConfirmCode::DataReceiveFail,
            0x0F => ConfirmCode::UploadImageFail,
            0x10 => ConfirmCode::DeleteFail,
            0x11 => ConfirmCode::ClearFail,
            0x13 => ConfirmCode::WrongPassword,
            0x15 => ConfirmCode::InvalidImage,
            0x17 => ConfirmCode::ResidualFingerprint,
            0x18 => ConfirmCode::FlashWriteError,
            0x19 => ConfirmCode::UndefinedError,
            0x1A => ConfirmCode::InvalidRegister,
            0x1B => ConfirmCode::RegisterConfigError,
            0x1C => ConfirmCode::InvalidNotepadPage,
            0x1D => ConfirmCode::PortOperationFail,
            0x1E => ConfirmCode::AutoEnrollFail,
            0x1F => ConfirmCode::LibraryFull,
            0x20 => ConfirmCode::InvalidAddress,
            0x21 => ConfirmCode::PasswordRequired,
            0x22 => ConfirmCode::TemplateNotEmpty,
            0x24 => ConfirmCode::LibraryEmpty,
            0x26 => ConfirmCode::DeviceTimeout,
            0x27 => ConfirmCode::AlreadyExists,
            0x29 => ConfirmCode::SensorHardwareError,
            0xFC => ConfirmCode::UnsupportedCommand,
            0xFD => ConfirmCode::HardwareError,
            0xFE => ConfirmCode::CommandFailed,
            other => ConfirmCode::Unknown(other),
        }
    }

    /// Returns the raw confirmation byte.
    pub fn as_u8(&self) -> u8 {
        match *self {
            ConfirmCode::Ok => 0x00,
            ConfirmCode::PacketReceiveError => 0x01,
            ConfirmCode::NoFinger => 0x02,
            ConfirmCode::EnrollImageFail => 0x03,
            ConfirmCode::ImageTooDry => 0x04,
            ConfirmCode::ImageTooWet => 0x05,
            ConfirmCode::ImageMessy => 0x06,
            ConfirmCode::FeatureTooFew => 0x07,
            ConfirmCode::FingerNotMatch => 0x08,
            ConfirmCode::NotFound => 0x09,
            ConfirmCode::MergeFail => 0x0A,
            ConfirmCode::InvalidPageId => 0x0B,
            ConfirmCode::ReadModelFail => 0x0C,
            ConfirmCode::UploadTemplateFail => 0x0D,
            ConfirmCode::DataReceiveFail => 0x0E,
            ConfirmCode::UploadImageFail => 0x0F,
            ConfirmCode::DeleteFail => 0x10,
            ConfirmCode::ClearFail => 0x11,
            ConfirmCode::WrongPassword => 0x13,
            ConfirmCode::InvalidImage => 0x15,
            ConfirmCode::ResidualFingerprint => 0x17,
            ConfirmCode::FlashWriteError => 0x18,
            ConfirmCode::UndefinedError => 0x19,
            ConfirmCode::InvalidRegister => 0x1A,
            ConfirmCode::RegisterConfigError => 0x1B,
            ConfirmCode::InvalidNotepadPage => 0x1C,
            ConfirmCode::PortOperationFail => 0x1D,
            ConfirmCode::AutoEnrollFail => 0x1E,
            ConfirmCode::LibraryFull => 0x1F,
            ConfirmCode::InvalidAddress => 0x20,
            ConfirmCode::PasswordRequired => 0x21,
            ConfirmCode::TemplateNotEmpty => 0x22,
            ConfirmCode::LibraryEmpty => 0x24,
            ConfirmCode::DeviceTimeout => 0x26,
            ConfirmCode::AlreadyExists => 0x27,
            ConfirmCode::SensorHardwareError => 0x29,
            ConfirmCode::UnsupportedCommand => 0xFC,
            ConfirmCode::HardwareError => 0xFD,
            ConfirmCode::CommandFailed => 0xFE,
            ConfirmCode::Unknown(raw) => raw,
        }
    }

    /// Returns a human-readable English description of the confirmation code.
    pub fn as_str(&self) -> &'static str {
        match self {
            ConfirmCode::Ok => "Command completed successfully",
            ConfirmCode::PacketReceiveError => "Packet communication error",
            ConfirmCode::NoFinger => "No finger detected on sensor",
            ConfirmCode::EnrollImageFail => "Failed to capture fingerprint image",
            ConfirmCode::ImageTooDry => "Fingerprint image too dry or too light",
            ConfirmCode::ImageTooWet => "Fingerprint image too wet or too blurry",
            ConfirmCode::ImageMessy => "Image is too messy or distorted",
            ConfirmCode::FeatureTooFew => "Too few feature points identified",
            ConfirmCode::FingerNotMatch => "Fingerprints do not match",
            ConfirmCode::NotFound => "Fingerprint not found in database",
            ConfirmCode::MergeFail => "Failed to merge character templates",
            ConfirmCode::InvalidPageId => "Invalid or out-of-range slot ID",
            ConfirmCode::ReadModelFail => "Failed to read template from flash memory",
            ConfirmCode::UploadTemplateFail => "Failed to upload template",
            ConfirmCode::DataReceiveFail => "Module failed to receive data package",
            ConfirmCode::UploadImageFail => "Failed to upload image",
            ConfirmCode::DeleteFail => "Failed to delete template",
            ConfirmCode::ClearFail => "Failed to clear fingerprint database",
            ConfirmCode::WrongPassword => "Incorrect module password",
            ConfirmCode::InvalidImage => "No valid primary image to generate image",
            ConfirmCode::ResidualFingerprint => {
                "Residual fingerprint or finger not moved between captures"
            }
            ConfirmCode::FlashWriteError => "Flash write error",
            ConfirmCode::UndefinedError => "Undefined error",
            ConfirmCode::InvalidRegister => "Invalid register number",
            ConfirmCode::RegisterConfigError => "Incorrect register configuration",
            ConfirmCode::InvalidNotepadPage => "Invalid notepad page number",
            ConfirmCode::PortOperationFail => "Failed to operate the communication port",
            ConfirmCode::AutoEnrollFail => "Automatic enrollment failed",
            ConfirmCode::LibraryFull => "Fingerprint library is full",
            ConfirmCode::InvalidAddress => "Incorrect device address",
            ConfirmCode::PasswordRequired => "Password must be verified first",
            ConfirmCode::TemplateNotEmpty => "Fingerprint template slot is not empty",
            ConfirmCode::LibraryEmpty => "Fingerprint library is empty",
            ConfirmCode::DeviceTimeout => "Operation timed out on the module",
            ConfirmCode::AlreadyExists => "Fingerprint already exists",
            ConfirmCode::SensorHardwareError => "Sensor hardware error",
            ConfirmCode::UnsupportedCommand => "Unsupported command",
            ConfirmCode::HardwareError => "Hardware error",
            ConfirmCode::CommandFailed => "Command execution failure",
            ConfirmCode::Unknown(_) => "Unknown confirmation code",
        }
    }
}

impl From<u8> for ConfirmCode {
    fn from(code: u8) -> Self {
        ConfirmCode::from_u8(code)
    }
}

impl From<ConfirmCode> for u8 {
    fn from(code: ConfirmCode) -> Self {
        code.as_u8()
    }
}

impl core::fmt::Display for ConfirmCode {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConfirmCode::Unknown(raw) => write!(f, "Unknown confirmation code 0x{:02X}", raw),
            other => write!(f, "{} (0x{:02X})", other.as_str(), other.as_u8()),
        }
    }
}

/// Builds a serial command packet into a caller-supplied fixed buffer without heap allocation.
///
/// Returns the slice of `out_buf` containing the constructed packet.
pub fn build_command_packet<'a>(
    address: [u8; 4],
    payload: &[u8],
    out_buf: &'a mut [u8],
) -> Result<&'a [u8], &'static str> {
    let packet_len = 11 + payload.len();
    if out_buf.len() < packet_len {
        return Err("Destination buffer too small");
    }

    out_buf[0] = START_CODE[0];
    out_buf[1] = START_CODE[1];
    out_buf[2..6].copy_from_slice(&address);
    out_buf[6] = PID_COMMAND;

    let length = (payload.len() + 2) as u16;
    out_buf[7] = (length >> 8) as u8;
    out_buf[8] = (length & 0xFF) as u8;

    out_buf[9..9 + payload.len()].copy_from_slice(payload);

    let mut checksum: u16 = PID_COMMAND as u16 + length;
    for &byte in payload {
        checksum = checksum.wrapping_add(byte as u16);
    }

    out_buf[packet_len - 2] = (checksum >> 8) as u8;
    out_buf[packet_len - 1] = (checksum & 0xFF) as u8;

    Ok(&out_buf[..packet_len])
}

/// A successfully framed and checksum-verified acknowledge packet.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AckPacket<'a> {
    /// Confirmation code reported by the module.
    pub code: ConfirmCode,
    /// Response data following the confirmation code (checksum excluded).
    pub data: &'a [u8],
    /// Byte range of `data` inside the raw buffer passed to [`parse_ack_packet`].
    pub data_range: Range<usize>,
    /// Index one past the last byte of this packet inside the raw buffer.
    pub end: usize,
}

/// Locates, frames and verifies the first acknowledge packet inside `raw`.
///
/// The parser is resilient to line noise:
///
/// - **Resynchronization**: any garbage preceding a `0xEF 0x01` header is skipped.
/// - **Dropped sync bytes**: if the stream begins with the module address (both sync
///   bytes lost) or with `0x01` + address (only `0xEF` lost), the packet is still
///   recognized. This works for any `address`, not only the default broadcast one.
///
/// Every candidate is validated by its checksum, so false header matches are discarded.
/// An `Err` means no complete valid packet is available yet; more bytes may fix it.
pub fn parse_ack_packet(raw: &[u8], address: [u8; 4]) -> Result<AckPacket<'_>, &'static str> {
    let mut last_err = if raw.is_empty() {
        "No data received"
    } else {
        "Missing synchronization header"
    };

    // Dropped sync bytes are only plausible at the very start of the stream.
    for skipped in [2usize, 1] {
        let sync = &START_CODE[skipped..];
        if raw.len() >= sync.len() + 5
            && raw.starts_with(sync)
            && raw[sync.len()..sync.len() + 4] == address
            && raw[sync.len() + 4] == PID_ACK
        {
            match try_frame(raw, sync.len() + 5) {
                Ok(packet) => return Ok(packet),
                Err(e) => last_err = e,
            }
        }
    }

    // Full header search anywhere in the buffer.
    let mut offset = 0;
    while offset < raw.len() {
        let rest = &raw[offset..];
        let header_match = rest
            .iter()
            .zip(START_CODE.iter().chain(address.iter()).chain(&[PID_ACK]))
            .all(|(a, b)| a == b);
        if header_match {
            if rest.len() < 9 {
                return Err("Header received but packet incomplete");
            }
            match try_frame(raw, offset + 7) {
                Ok(packet) => return Ok(packet),
                Err(e) => last_err = e,
            }
        }
        offset += 1;
    }

    Err(last_err)
}

/// Frames a packet whose 2-byte length field starts at `len_pos` (right after the PID byte).
fn try_frame(raw: &[u8], len_pos: usize) -> Result<AckPacket<'_>, &'static str> {
    if raw.len() < len_pos + 2 {
        return Err("Header received but packet incomplete");
    }
    let length = ((raw[len_pos] as u16) << 8) | (raw[len_pos + 1] as u16);
    if length < 3 {
        return Err("Invalid packet length field");
    }

    let payload_offset = len_pos + 2;
    let total_len = payload_offset + length as usize;
    if raw.len() < total_len {
        return Err("Packet payload incomplete");
    }

    let packet_data = &raw[payload_offset..total_len - 2];
    let received_checksum = ((raw[total_len - 2] as u16) << 8) | (raw[total_len - 1] as u16);

    let mut calculated_checksum: u16 = (PID_ACK as u16).wrapping_add(length);
    for &byte in packet_data {
        calculated_checksum = calculated_checksum.wrapping_add(byte as u16);
    }

    if calculated_checksum != received_checksum {
        return Err("Checksum mismatch");
    }

    Ok(AckPacket {
        code: ConfirmCode::from_u8(packet_data[0]),
        data: &packet_data[1..],
        data_range: payload_offset + 1..total_len - 2,
        end: total_len,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const OK_ACK: [u8; 12] = [
        0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x03, 0x00, 0x00, 0x0A,
    ];

    #[test]
    fn test_build_command_packet() {
        let mut buf = [0u8; 32];
        let payload = [CMD_GET_IMAGE];
        let packet = build_command_packet(DEFAULT_ADDRESS, &payload, &mut buf).unwrap();

        assert_eq!(&packet[0..2], &START_CODE);
        assert_eq!(&packet[2..6], &DEFAULT_ADDRESS);
        assert_eq!(packet[6], PID_COMMAND);
        assert_eq!(packet[7..9], [0x00, 0x03]); // length = payload.len() + 2 = 3
        assert_eq!(packet[9], CMD_GET_IMAGE);

        // Checksum: PID (0x01) + Length (0x00, 0x03) + CMD (0x01) = 0x05
        assert_eq!(packet[10..12], [0x00, 0x05]);
    }

    #[test]
    fn test_parse_ack_packet() {
        let packet = parse_ack_packet(&OK_ACK, DEFAULT_ADDRESS).unwrap();
        assert_eq!(packet.code, ConfirmCode::Ok);
        assert!(packet.data.is_empty());
        assert_eq!(packet.end, OK_ACK.len());
    }

    #[test]
    fn test_parse_ack_packet_with_dropped_sync_header() {
        // Both sync bytes dropped by Linux TTY
        let packet = parse_ack_packet(&OK_ACK[2..], DEFAULT_ADDRESS).unwrap();
        assert_eq!(packet.code, ConfirmCode::Ok);
        assert!(packet.data.is_empty());
    }

    #[test]
    fn test_parse_ack_packet_with_single_dropped_sync_byte() {
        let packet = parse_ack_packet(&OK_ACK[1..], DEFAULT_ADDRESS).unwrap();
        assert_eq!(packet.code, ConfirmCode::Ok);
    }

    #[test]
    fn test_parse_ack_packet_dropped_sync_custom_address() {
        let addr = [0x12, 0x34, 0x56, 0x78];
        let raw = [0x12, 0x34, 0x56, 0x78, 0x07, 0x00, 0x03, 0x00, 0x00, 0x0A];
        let packet = parse_ack_packet(&raw, addr).unwrap();
        assert_eq!(packet.code, ConfirmCode::Ok);
        // Wrong address must not match.
        assert!(parse_ack_packet(&raw, DEFAULT_ADDRESS).is_err());
    }

    #[test]
    fn test_parse_ack_packet_resync_after_garbage() {
        let mut raw = [0u8; 17];
        raw[..5].copy_from_slice(&[0x00, 0xEF, 0x13, 0xFF, 0x42]);
        raw[5..].copy_from_slice(&OK_ACK);
        let packet = parse_ack_packet(&raw, DEFAULT_ADDRESS).unwrap();
        assert_eq!(packet.code, ConfirmCode::Ok);
        assert_eq!(packet.end, raw.len());
    }

    #[test]
    fn test_parse_ack_packet_incomplete() {
        assert!(parse_ack_packet(&OK_ACK[..8], DEFAULT_ADDRESS).is_err());
        assert!(parse_ack_packet(&OK_ACK[..11], DEFAULT_ADDRESS).is_err());
    }

    #[test]
    fn test_parse_ack_packet_with_data_and_trailing_packet() {
        // Template count = 0x0005, followed by a second packet.
        let mut raw = [0u8; 26];
        raw[..14].copy_from_slice(&[
            0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x05, 0x00, 0x00, 0x05, 0x00, 0x11,
        ]);
        raw[14..].copy_from_slice(&OK_ACK);
        let packet = parse_ack_packet(&raw, DEFAULT_ADDRESS).unwrap();
        assert_eq!(packet.code, ConfirmCode::Ok);
        assert_eq!(packet.data, &[0x00, 0x05]);
        assert_eq!(packet.data_range, 10..12);
        assert_eq!(packet.end, 14);
    }

    #[test]
    fn test_confirm_code_roundtrip() {
        for raw in 0..=u8::MAX {
            assert_eq!(ConfirmCode::from_u8(raw).as_u8(), raw);
        }
        assert_eq!(ConfirmCode::from_u8(0x99), ConfirmCode::Unknown(0x99));
    }
}
