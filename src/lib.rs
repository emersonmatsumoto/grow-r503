#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]
//! # grow-r503
//!
//! A platform-agnostic, zero-allocation driver for the **Grow R503** capacitive fingerprint sensor
//! supporting the ZhianTec ZFM / Syno serial protocol and the circular RGB Aura LED ring.
//!
//! ## Features
//!
//! - **Full R503 Instruction Set**: Image capture, template extraction, 1:1 matching, 1:N searching,
//!   model fusion, flash storage, deletion, index table reading, system parameter inspection and
//!   on-module automatic enrollment.
//! - **Circular Aura RGB LED Ring**: Complete control over animation modes (`Breathing`, `Flashing`,
//!   `AlwaysOn`, `GradualOn`), colors, speeds, and cycles.
//! - **Platform Agnostic (`no_std` compatible)**: Implements fixed-stack packet framing with zero heap
//!   allocation. The `std` feature is enabled by default; disable default features to use the crate
//!   on bare-metal microcontrollers (ESP32, RP2040, STM32).
//! - **Resilient framing**: Resynchronizes on the `0xEF 0x01` header after line noise and recovers
//!   from dropped synchronization bytes caused by TTY line disciplines.
//!
//! ## Cargo features
//!
//! | Feature       | Default | Description |
//! |:--------------|:-------:|:------------|
//! | `std`         | yes     | `std::error::Error` impls and the [`IoTransport`] adapter for `std::io` streams. |
//! | `embedded-io` | no      | The [`EmbeddedIoTransport`] adapter for `embedded_io::{Read, Write}` UARTs. |
//!
//! ## Example (Standard Library on Linux / SBC)
//!
//! ```no_run
//! use grow_r503::{GrowR503, IoTransport};
//! use std::time::Duration;
//!
//! fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let port = serialport::new("/dev/ttyS0", 57600)
//!         .timeout(Duration::from_millis(2000))
//!         .open()?;
//!
//!     let mut sensor = GrowR503::new(IoTransport::new(port));
//!     sensor.verify_password([0x00, 0x00, 0x00, 0x00])?;
//!     sensor.led_standby()?;
//!
//!     // Check registered fingerprint count
//!     let count = sensor.get_template_count()?;
//!     println!("Enrolled fingerprints: {}", count);
//!     Ok(())
//! }
//! ```

pub mod led;
pub mod protocol;
pub mod transport;

pub use led::{LedColor, LedMode, ParseLedError};
pub use protocol::ConfirmCode;
#[cfg(feature = "embedded-io")]
pub use transport::EmbeddedIoTransport;
#[cfg(feature = "std")]
pub use transport::IoTransport;
pub use transport::{SystemParameters, Transport};

use core::ops::Range;
use protocol::*;

/// Maximum number of non-empty reads spent looking for a single acknowledge packet
/// before giving up (protects against a continuous stream of garbage).
const MAX_DATA_READS: u32 = 64;

/// Driver error types.
///
/// This enum is `#[non_exhaustive]`: new variants may be added in minor releases.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error<E> {
    /// Transport I/O failure.
    Transport(E),
    /// Protocol packet encoding/decoding error (malformed, truncated or unexpected response).
    Protocol(&'static str),
    /// Sensor returned a failure confirmation code.
    Device(ConfirmCode),
    /// No response was received from the sensor before the transport timed out.
    Timeout,
}

impl<E> From<ConfirmCode> for Error<E> {
    fn from(code: ConfirmCode) -> Self {
        Error::Device(code)
    }
}

impl<E: core::fmt::Display> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Transport(e) => write!(f, "Transport error: {}", e),
            Error::Protocol(msg) => write!(f, "Protocol error: {}", msg),
            Error::Device(code) => write!(f, "Device error: {}", code),
            Error::Timeout => write!(f, "Operation timed out"),
        }
    }
}

#[cfg(feature = "std")]
impl<E: std::error::Error + 'static> std::error::Error for Error<E> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Transport(e) => Some(e),
            _ => None,
        }
    }
}

/// Options for [`GrowR503::auto_enroll_with`] (instruction `0x31`, `AutoEnroll`).
///
/// Construct with [`Default::default`] and adjust the fields you need.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct AutoEnrollOptions {
    /// Keep the LED ring off while capturing (parameter bit 0).
    pub led_off_during_capture: bool,
    /// Skip image pre-processing (parameter bit 1).
    pub skip_preprocessing: bool,
    /// Allow overwriting an occupied slot (parameter bit 3).
    pub allow_overwrite: bool,
    /// Reject the enrollment if the fingerprint already exists in the library (parameter bit 4).
    pub reject_duplicates: bool,
    /// Require the finger to be lifted between samples (parameter bit 5, inverted).
    pub require_finger_lift: bool,
    /// Number of consecutive transport read timeouts tolerated while waiting for the next
    /// status packet (the module waits for the user to place / lift the finger).
    /// With a 1 s transport timeout, the default of 30 allows ~30 s per step.
    pub max_idle_reads: u32,
}

impl Default for AutoEnrollOptions {
    fn default() -> Self {
        Self {
            led_off_during_capture: false,
            skip_preprocessing: false,
            allow_overwrite: false,
            reject_duplicates: false,
            require_finger_lift: true,
            max_idle_reads: 30,
        }
    }
}

impl AutoEnrollOptions {
    /// Encodes the options into the 16-bit `AutoEnroll` parameter word.
    ///
    /// Bit 2 (step status reporting) is always cleared so the module reports every step.
    fn to_param(self) -> u16 {
        let mut p = 0u16;
        if self.led_off_during_capture {
            p |= 1 << 0;
        }
        if self.skip_preprocessing {
            p |= 1 << 1;
        }
        if self.allow_overwrite {
            p |= 1 << 3;
        }
        if self.reject_duplicates {
            p |= 1 << 4;
        }
        if !self.require_finger_lift {
            p |= 1 << 5;
        }
        p
    }
}

/// Progress reported by the module during [`GrowR503::auto_enroll_with`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum AutoEnrollStep {
    /// Parameters validated by the module (step `0x00`).
    Validated,
    /// Fingerprint image captured for sample `n` (step `0x01`).
    ImageCaptured(u8),
    /// Feature file generated for sample `n` (step `0x02`).
    FeatureGenerated(u8),
    /// Finger lifted after sample `n` (step `0x03`).
    FingerLifted(u8),
    /// Samples merged into a template (step `0x04`).
    TemplateMerged,
    /// Duplicate check finished (step `0x05`).
    DuplicateChecked,
    /// Template stored in flash; enrollment complete (step `0x06`).
    Stored,
    /// Unrecognized step code, raw values preserved.
    Other { step: u8, param: u8 },
}

impl AutoEnrollStep {
    fn from_raw(step: u8, param: u8) -> Self {
        match step {
            0x00 => AutoEnrollStep::Validated,
            0x01 => AutoEnrollStep::ImageCaptured(param),
            0x02 => AutoEnrollStep::FeatureGenerated(param),
            0x03 => AutoEnrollStep::FingerLifted(param),
            0x04 => AutoEnrollStep::TemplateMerged,
            0x05 => AutoEnrollStep::DuplicateChecked,
            0x06 => AutoEnrollStep::Stored,
            _ => AutoEnrollStep::Other { step, param },
        }
    }
}

/// Driver interface for the Grow R503 biometric fingerprint sensor.
pub struct GrowR503<T> {
    transport: T,
    address: [u8; 4],
}

impl<T: Transport> GrowR503<T> {
    /// Creates a new driver instance using the default broadcast address (`0xFFFFFFFF`).
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            address: DEFAULT_ADDRESS,
        }
    }

    /// Creates a driver instance with a customized 32-bit module address.
    pub fn with_address(transport: T, address: [u8; 4]) -> Self {
        Self { transport, address }
    }

    /// Borrows the underlying transport stream.
    pub fn transport(&self) -> &T {
        &self.transport
    }

    /// Mutably borrows the underlying transport stream.
    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    /// Consumes the driver, returning the underlying transport.
    pub fn release(self) -> T {
        self.transport
    }

    /// Frames and transmits a command packet.
    fn send_command(&mut self, payload: &[u8]) -> Result<(), Error<T::Error>> {
        let mut tx_buf = [0u8; 64];
        let packet =
            build_command_packet(self.address, payload, &mut tx_buf).map_err(Error::Protocol)?;

        self.transport.write_all(packet).map_err(Error::Transport)?;
        self.transport.flush().map_err(Error::Transport)
    }

    /// Reads from the transport until a complete, valid acknowledge packet is present in
    /// `rx[..*filled]`.
    ///
    /// Returns the confirmation code, the data range inside `rx`, and the end offset of the
    /// packet so callers expecting several packets can consume it and keep the remainder.
    fn receive_ack(
        &mut self,
        rx: &mut [u8],
        filled: &mut usize,
        max_idle_reads: u32,
    ) -> Result<(ConfirmCode, Range<usize>, usize), Error<T::Error>> {
        let mut idle_reads = 0u32;
        let mut data_reads = 0u32;

        loop {
            let last_err = match parse_ack_packet(&rx[..*filled], self.address) {
                Ok(packet) => return Ok((packet.code, packet.data_range, packet.end)),
                Err(e) => e,
            };

            if data_reads >= MAX_DATA_READS {
                return Err(Error::Protocol(last_err));
            }

            if *filled == rx.len() {
                // Buffer full without a valid packet: drop everything before the last
                // possible header start so a packet in progress can still complete.
                let keep_from = rx[1..*filled]
                    .iter()
                    .rposition(|&b| b == START_CODE[0])
                    .map(|i| i + 1)
                    .unwrap_or(*filled);
                rx.copy_within(keep_from..*filled, 0);
                *filled -= keep_from;
            }

            match self.transport.read(&mut rx[*filled..]) {
                Ok(0) => {
                    if idle_reads >= max_idle_reads {
                        return if *filled == 0 {
                            Err(Error::Timeout)
                        } else {
                            Err(Error::Protocol(last_err))
                        };
                    }
                    idle_reads += 1;
                }
                Ok(n) => {
                    *filled += n;
                    idle_reads = 0;
                    data_reads += 1;
                }
                Err(e) => return Err(Error::Transport(e)),
            }
        }
    }

    /// Sends a raw command payload and waits for the acknowledge response packet.
    fn execute_command<'a>(
        &mut self,
        payload: &[u8],
        rx_buf: &'a mut [u8],
    ) -> Result<(ConfirmCode, &'a [u8]), Error<T::Error>> {
        self.send_command(payload)?;
        let mut filled = 0;
        let (code, data, _) = self.receive_ack(rx_buf, &mut filled, 0)?;
        Ok((code, &rx_buf[data]))
    }

    /// Sends a command whose only meaningful response is the confirmation code.
    fn simple_command(&mut self, payload: &[u8]) -> Result<(), Error<T::Error>> {
        let mut rx = [0u8; 32];
        let (code, _) = self.execute_command(payload, &mut rx)?;
        ok_or_device(code)
    }

    /// Verifies the module access password (default: `0x00000000`).
    pub fn verify_password(&mut self, password: [u8; 4]) -> Result<(), Error<T::Error>> {
        self.simple_command(&[
            CMD_VERIFY_PASSWORD,
            password[0],
            password[1],
            password[2],
            password[3],
        ])
    }

    /// Sets a new 4-byte module access password.
    pub fn set_password(&mut self, new_password: [u8; 4]) -> Result<(), Error<T::Error>> {
        self.simple_command(&[
            CMD_SET_PASSWORD,
            new_password[0],
            new_password[1],
            new_password[2],
            new_password[3],
        ])
    }

    /// Sets a new 4-byte device communication address.
    pub fn set_address(&mut self, new_address: [u8; 4]) -> Result<(), Error<T::Error>> {
        self.simple_command(&[
            CMD_SET_ADDRESS,
            new_address[0],
            new_address[1],
            new_address[2],
            new_address[3],
        ])?;
        self.address = new_address;
        Ok(())
    }

    /// Configures the Aura RGB LED ring effect.
    ///
    /// - `mode`: Animation mode (`Breathing`, `Flashing`, `AlwaysOn`, etc.).
    /// - `color`: Target RGB color.
    /// - `speed`: Animation frequency/speed (0 to 255).
    /// - `times`: Repetitions (0 = continuous infinite animation).
    pub fn set_led(
        &mut self,
        mode: LedMode,
        color: LedColor,
        speed: u8,
        times: u8,
    ) -> Result<(), Error<T::Error>> {
        self.simple_command(&[CMD_AURA_LED_CONFIG, mode as u8, speed, color as u8, times])
    }

    /// Convenient helper: Gentle blue breathing standby glow.
    pub fn led_standby(&mut self) -> Result<(), Error<T::Error>> {
        self.set_led(LedMode::Breathing, LedColor::Blue, 0x80, 0)
    }

    /// Convenient helper: Quick cyan flashing indicating biometric template generation.
    pub fn led_processing(&mut self) -> Result<(), Error<T::Error>> {
        self.set_led(LedMode::Flashing, LedColor::Cyan, 0x20, 2)
    }

    /// Convenient helper: Green flashing confirming verified match.
    pub fn led_success(&mut self) -> Result<(), Error<T::Error>> {
        self.set_led(LedMode::Flashing, LedColor::Green, 0x30, 3)
    }

    /// Convenient helper: Solid green steady glow indicating unlocked state.
    pub fn led_unlocked_continuous(&mut self) -> Result<(), Error<T::Error>> {
        self.set_led(LedMode::AlwaysOn, LedColor::Green, 0x00, 0)
    }

    /// Convenient helper: Rapid green flashing for disk / storage I/O activity.
    pub fn led_io_active(&mut self) -> Result<(), Error<T::Error>> {
        self.set_led(LedMode::Flashing, LedColor::Green, 0x20, 0)
    }

    /// Convenient helper: Fast red flashes indicating failure or access denied.
    pub fn led_error(&mut self) -> Result<(), Error<T::Error>> {
        self.set_led(LedMode::Flashing, LedColor::Red, 0x20, 3)
    }

    /// Turns off the LED ring.
    pub fn led_off(&mut self) -> Result<(), Error<T::Error>> {
        self.set_led(LedMode::AlwaysOff, LedColor::Blue, 0x00, 0)
    }

    /// Captures a raw fingerprint image from the sensor surface into the internal image buffer.
    ///
    /// Returns `Ok(true)` when a finger is detected and captured, or `Ok(false)` when no finger is present.
    pub fn get_image(&mut self) -> Result<bool, Error<T::Error>> {
        let mut rx = [0u8; 32];
        let (code, _) = self.execute_command(&[CMD_GET_IMAGE], &mut rx)?;
        match code {
            ConfirmCode::Ok => Ok(true),
            ConfirmCode::NoFinger | ConfirmCode::EnrollImageFail => Ok(false),
            other => Err(Error::Device(other)),
        }
    }

    /// Converts the raw image in the image buffer into a feature template stored in `buffer_id` (1 or 2).
    pub fn image_to_template(&mut self, buffer_id: u8) -> Result<(), Error<T::Error>> {
        if buffer_id != 1 && buffer_id != 2 {
            return Err(Error::Protocol("Buffer ID must be 1 or 2"));
        }
        self.simple_command(&[CMD_IMAGE_TO_TEMPLATE, buffer_id])
    }

    /// Performs a precise 1:1 match between the templates currently held in CharBuffer1 and CharBuffer2.
    ///
    /// Returns the matching similarity confidence score on success.
    pub fn match_templates(&mut self) -> Result<u16, Error<T::Error>> {
        let mut rx = [0u8; 32];
        let (code, data) = self.execute_command(&[CMD_MATCH], &mut rx)?;
        ok_or_device(code)?;
        be_u16(data).ok_or(Error::Protocol("Match response data truncated"))
    }

    /// Fuses character templates in CharBuffer1 and CharBuffer2 into a unified registered model.
    pub fn create_model(&mut self) -> Result<(), Error<T::Error>> {
        self.simple_command(&[CMD_REG_MODEL])
    }

    /// Saves the template currently held in `buffer_id` into non-volatile flash memory at `page_id`.
    pub fn store_model(&mut self, page_id: u16, buffer_id: u8) -> Result<(), Error<T::Error>> {
        let [hi, lo] = page_id.to_be_bytes();
        self.simple_command(&[CMD_STORE, buffer_id, hi, lo])
    }

    /// Loads a template from flash memory slot `page_id` into `buffer_id` (1 or 2).
    pub fn load_model(&mut self, page_id: u16, buffer_id: u8) -> Result<(), Error<T::Error>> {
        let [hi, lo] = page_id.to_be_bytes();
        self.simple_command(&[CMD_LOAD_TEMPLATE, buffer_id, hi, lo])
    }

    /// Searches the on-chip library for a template matching `buffer_id`.
    ///
    /// Returns `Ok(Some((slot_id, score)))` on match, or `Ok(None)` if no match is found.
    pub fn search(
        &mut self,
        buffer_id: u8,
        start_page: u16,
        page_count: u16,
    ) -> Result<Option<(u16, u16)>, Error<T::Error>> {
        let mut rx = [0u8; 32];
        let [start_hi, start_lo] = start_page.to_be_bytes();
        let [count_hi, count_lo] = page_count.to_be_bytes();
        let payload = [
            CMD_SEARCH, buffer_id, start_hi, start_lo, count_hi, count_lo,
        ];
        let (code, data) = self.execute_command(&payload, &mut rx)?;
        match code {
            ConfirmCode::Ok => {
                if data.len() >= 4 {
                    let page_id = u16::from_be_bytes([data[0], data[1]]);
                    let score = u16::from_be_bytes([data[2], data[3]]);
                    Ok(Some((page_id, score)))
                } else {
                    Err(Error::Protocol("Search response data truncated"))
                }
            }
            ConfirmCode::NotFound => Ok(None),
            other => Err(Error::Device(other)),
        }
    }

    /// Returns the total number of enrolled fingerprint templates stored in flash memory.
    pub fn get_template_count(&mut self) -> Result<u16, Error<T::Error>> {
        let mut rx = [0u8; 32];
        let (code, data) = self.execute_command(&[CMD_TEMPLATE_COUNT], &mut rx)?;
        ok_or_device(code)?;
        be_u16(data).ok_or(Error::Protocol("Template count response data truncated"))
    }

    /// Deletes `count` templates starting at flash slot `start_page`.
    pub fn delete_models(&mut self, start_page: u16, count: u16) -> Result<(), Error<T::Error>> {
        let [start_hi, start_lo] = start_page.to_be_bytes();
        let [count_hi, count_lo] = count.to_be_bytes();
        self.simple_command(&[CMD_DELETE_TEMPLATE, start_hi, start_lo, count_hi, count_lo])
    }

    /// Clears the entire database of stored fingerprint templates.
    pub fn empty_database(&mut self) -> Result<(), Error<T::Error>> {
        self.simple_command(&[CMD_EMPTY_DATABASE])
    }

    /// Reads system parameters (capacity, baud rate, security level, packet size).
    pub fn read_system_parameters(&mut self) -> Result<SystemParameters, Error<T::Error>> {
        let mut rx = [0u8; 32];
        let (code, data) = self.execute_command(&[CMD_READ_SYS_PARAMS], &mut rx)?;
        ok_or_device(code)?;
        SystemParameters::from_bytes(data)
            .ok_or(Error::Protocol("System parameters response data truncated"))
    }

    /// Writes 32 bytes of arbitrary user data into a 32-byte notepad page (0..=15).
    ///
    /// The Grow R503 provides 512 bytes of non-volatile user flash memory divided into
    /// 16 pages of 32 bytes each. Writing to a page overwrites its previous content.
    pub fn write_notepad(&mut self, page: u8, data: &[u8; 32]) -> Result<(), Error<T::Error>> {
        if page > 15 {
            return Err(Error::Device(ConfirmCode::InvalidNotepadPage));
        }
        let mut payload = [0u8; 34];
        payload[0] = CMD_WRITE_NOTEPAD;
        payload[1] = page;
        payload[2..34].copy_from_slice(data);
        self.simple_command(&payload)
    }

    /// Reads 32 bytes of user data from a 32-byte notepad page (0..=15).
    ///
    /// The Grow R503 provides 512 bytes of non-volatile user flash memory divided into
    /// 16 pages of 32 bytes each.
    pub fn read_notepad(&mut self, page: u8) -> Result<[u8; 32], Error<T::Error>> {
        if page > 15 {
            return Err(Error::Device(ConfirmCode::InvalidNotepadPage));
        }
        // Buffer sized for the ACK packet (sync + address + PID + length + code + 32-byte data + checksum = 44 bytes)
        let mut rx = [0u8; 64];
        let (code, data) = self.execute_command(&[CMD_READ_NOTEPAD, page], &mut rx)?;
        ok_or_device(code)?;
        if data.len() < 32 {
            return Err(Error::Protocol("Notepad read data truncated"));
        }
        let mut out = [0u8; 32];
        out.copy_from_slice(&data[..32]);
        Ok(out)
    }

    /// Reads the template index table page (0 to 3), returning a 32-byte bitmap (256 slots).
    ///
    /// Bit value 1 indicates the slot is occupied, 0 indicates empty.
    pub fn read_index_table(&mut self, page: u8) -> Result<[u8; 32], Error<T::Error>> {
        let mut rx = [0u8; 64];
        let (code, data) = self.execute_command(&[CMD_READ_INDEX_TABLE, page], &mut rx)?;
        ok_or_device(code)?;
        let bytes = data
            .get(..32)
            .ok_or(Error::Protocol("Index table response data truncated"))?;
        let mut table = [0u8; 32];
        table.copy_from_slice(bytes);
        Ok(table)
    }

    /// Generates a hardware 32-bit pseudo-random number from the sensor's DSP.
    pub fn get_random_number(&mut self) -> Result<u32, Error<T::Error>> {
        let mut rx = [0u8; 32];
        let (code, data) = self.execute_command(&[CMD_GET_RANDOM_CODE], &mut rx)?;
        ok_or_device(code)?;
        match data {
            [a, b, c, d, ..] => Ok(u32::from_be_bytes([*a, *b, *c, *d])),
            _ => Err(Error::Protocol("Random number response data truncated")),
        }
    }

    /// Runs the module's autonomous enrollment (`0x31`) with default [`AutoEnrollOptions`].
    ///
    /// The user must place the finger `sample_count` times; the module captures, merges and
    /// stores the template at `page_id` by itself. Blocks until the module reports the final
    /// result. See [`auto_enroll_with`](Self::auto_enroll_with) for options and progress reporting.
    pub fn auto_enroll(&mut self, page_id: u16, sample_count: u8) -> Result<(), Error<T::Error>> {
        self.auto_enroll_with(page_id, sample_count, AutoEnrollOptions::default(), |_| {})
    }

    /// Runs the module's autonomous enrollment (`0x31`), reporting each step to `progress`.
    ///
    /// The module sends one acknowledge packet per step (image captured, feature generated,
    /// finger lifted, merged, duplicate-checked, stored). All of them are consumed here, so
    /// the next command starts on a clean stream. Returns `Ok(())` once the template is stored,
    /// or [`Error::Device`] with the code of the failing step.
    pub fn auto_enroll_with<F>(
        &mut self,
        page_id: u16,
        sample_count: u8,
        options: AutoEnrollOptions,
        mut progress: F,
    ) -> Result<(), Error<T::Error>>
    where
        F: FnMut(AutoEnrollStep),
    {
        let [id_hi, id_lo] = page_id.to_be_bytes();
        let [param_hi, param_lo] = options.to_param().to_be_bytes();
        self.send_command(&[
            CMD_AUTO_ENROLL,
            id_hi,
            id_lo,
            sample_count,
            param_hi,
            param_lo,
        ])?;

        let mut rx = [0u8; 64];
        let mut filled = 0;
        loop {
            let (code, data, end) =
                self.receive_ack(&mut rx, &mut filled, options.max_idle_reads)?;
            let status = match rx[data] {
                [step, param, ..] => Some((step, param)),
                _ => None,
            };
            // Consume this packet, keeping any bytes of the next one.
            rx.copy_within(end..filled, 0);
            filled -= end;

            ok_or_device(code)?;
            let (step, param) =
                status.ok_or(Error::Protocol("AutoEnroll status response truncated"))?;
            let step = AutoEnrollStep::from_raw(step, param);
            progress(step);
            if step == AutoEnrollStep::Stored {
                return Ok(());
            }
        }
    }
}

fn ok_or_device<E>(code: ConfirmCode) -> Result<(), Error<E>> {
    if code == ConfirmCode::Ok {
        Ok(())
    } else {
        Err(Error::Device(code))
    }
}

fn be_u16(data: &[u8]) -> Option<u16> {
    match data {
        [hi, lo, ..] => Some(u16::from_be_bytes([*hi, *lo])),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// In-memory transport replaying scripted response chunks.
    struct MockTransport<'a> {
        chunks: &'a [&'a [u8]],
        next: usize,
        offset: usize,
        written: usize,
    }

    impl<'a> MockTransport<'a> {
        fn new(chunks: &'a [&'a [u8]]) -> Self {
            Self {
                chunks,
                next: 0,
                offset: 0,
                written: 0,
            }
        }
    }

    impl Transport for MockTransport<'_> {
        type Error = ();

        fn write_all(&mut self, data: &[u8]) -> Result<(), ()> {
            self.written += data.len();
            Ok(())
        }

        fn flush(&mut self) -> Result<(), ()> {
            Ok(())
        }

        fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
            let Some(chunk) = self.chunks.get(self.next) else {
                return Ok(0);
            };
            let rest = &chunk[self.offset..];
            let n = rest.len().min(buf.len());
            buf[..n].copy_from_slice(&rest[..n]);
            self.offset += n;
            if self.offset == chunk.len() {
                self.next += 1;
                self.offset = 0;
            }
            Ok(n)
        }
    }

    /// Builds an ACK packet (default address) into `out`, returning its length.
    fn ack(code: u8, data: &[u8], out: &mut [u8]) -> usize {
        let length = (data.len() + 3) as u16;
        out[..7].copy_from_slice(&[0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, PID_ACK]);
        out[7..9].copy_from_slice(&length.to_be_bytes());
        out[9] = code;
        out[10..10 + data.len()].copy_from_slice(data);
        let mut sum = (PID_ACK as u16) + length + code as u16;
        for &b in data {
            sum = sum.wrapping_add(b as u16);
        }
        let end = 10 + data.len();
        out[end..end + 2].copy_from_slice(&sum.to_be_bytes());
        end + 2
    }

    #[test]
    fn timeout_when_no_response() {
        let mut sensor = GrowR503::new(MockTransport::new(&[]));
        assert_eq!(sensor.verify_password([0; 4]), Err(Error::Timeout));
    }

    #[test]
    fn truncated_data_is_protocol_error() {
        let mut buf = [0u8; 32];
        let n = ack(0x00, &[0x01], &mut buf);
        let chunks: [&[u8]; 1] = [&buf[..n]];
        let mut sensor = GrowR503::new(MockTransport::new(&chunks));
        assert!(matches!(
            sensor.get_template_count(),
            Err(Error::Protocol(_))
        ));
    }

    #[test]
    fn split_reads_and_garbage_are_handled() {
        let mut buf = [0u8; 32];
        let n = ack(0x00, &[0x00, 0x07], &mut buf);
        let chunks: [&[u8]; 3] = [&[0x00, 0x13], &buf[..5], &buf[5..n]];
        let mut sensor = GrowR503::new(MockTransport::new(&chunks));
        assert_eq!(sensor.get_template_count(), Ok(7));
    }

    #[test]
    fn device_error_is_reported() {
        let mut buf = [0u8; 32];
        let n = ack(0x13, &[], &mut buf);
        let chunks: [&[u8]; 1] = [&buf[..n]];
        let mut sensor = GrowR503::new(MockTransport::new(&chunks));
        assert_eq!(
            sensor.verify_password([0; 4]),
            Err(Error::Device(ConfirmCode::WrongPassword))
        );
    }

    #[test]
    fn auto_enroll_consumes_all_status_packets() {
        let steps: [[u8; 2]; 8] = [
            [0x00, 0x00],
            [0x01, 0x01],
            [0x02, 0x01],
            [0x03, 0x01],
            [0x01, 0x02],
            [0x02, 0x02],
            [0x04, 0xF0],
            [0x06, 0xF2],
        ];
        let mut stream = [0u8; 9 * 14];
        let mut len = 0;
        for s in &steps {
            len += ack(0x00, s, &mut stream[len..]);
        }
        // Trailing bytes after the final packet must not affect the result.
        len += ack(0x00, &[0x00, 0x01], &mut stream[len..]);

        // Deliver everything in two arbitrary chunks.
        let chunks: [&[u8]; 2] = [&stream[..20], &stream[20..len]];
        let mut sensor = GrowR503::new(MockTransport::new(&chunks));

        let mut seen = 0;
        sensor
            .auto_enroll_with(3, 2, AutoEnrollOptions::default(), |_| seen += 1)
            .unwrap();
        assert_eq!(seen, steps.len());
    }

    #[test]
    fn auto_enroll_reports_failing_step() {
        let mut stream = [0u8; 64];
        let mut len = ack(0x00, &[0x00, 0x00], &mut stream);
        len += ack(0x27, &[0x05, 0xF1], &mut stream[len..]);
        let chunks: [&[u8]; 1] = [&stream[..len]];
        let mut sensor = GrowR503::new(MockTransport::new(&chunks));
        assert_eq!(
            sensor.auto_enroll(1, 2),
            Err(Error::Device(ConfirmCode::AlreadyExists))
        );
    }

    #[test]
    fn write_notepad_invalid_page_bounds() {
        let mut sensor = GrowR503::new(MockTransport::new(&[]));
        assert_eq!(
            sensor.write_notepad(16, &[0xAA; 32]),
            Err(Error::Device(ConfirmCode::InvalidNotepadPage))
        );
    }

    #[test]
    fn read_notepad_invalid_page_bounds() {
        let mut sensor = GrowR503::new(MockTransport::new(&[]));
        assert_eq!(
            sensor.read_notepad(16),
            Err(Error::Device(ConfirmCode::InvalidNotepadPage))
        );
    }

    #[test]
    fn write_notepad_success() {
        let mut buf = [0u8; 32];
        let n = ack(0x00, &[], &mut buf);
        let chunks: [&[u8]; 1] = [&buf[..n]];
        let mut sensor = GrowR503::new(MockTransport::new(&chunks));
        assert_eq!(sensor.write_notepad(0, &[0x42; 32]), Ok(()));
    }

    #[test]
    fn read_notepad_success() {
        let expected = [0x5A; 32];
        let mut buf = [0u8; 64];
        let n = ack(0x00, &expected, &mut buf);
        let chunks: [&[u8]; 1] = [&buf[..n]];
        let mut sensor = GrowR503::new(MockTransport::new(&chunks));
        assert_eq!(sensor.read_notepad(1), Ok(expected));
    }

    #[test]
    fn read_notepad_truncated() {
        let short_data = [0x5A; 16];
        let mut buf = [0u8; 64];
        let n = ack(0x00, &short_data, &mut buf);
        let chunks: [&[u8]; 1] = [&buf[..n]];
        let mut sensor = GrowR503::new(MockTransport::new(&chunks));
        assert!(matches!(sensor.read_notepad(0), Err(Error::Protocol(_))));
    }
}
