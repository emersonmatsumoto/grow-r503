//! Abstraction layer for UART communication.

/// Generic serial byte transport trait.
///
/// # Read contract
///
/// [`Transport::read`] should block until at least one byte is available **or** the
/// transport's own read timeout elapses. On timeout it should return `Ok(0)`, which the
/// driver reports as [`Error::Timeout`](crate::Error::Timeout). Configure the timeout so
/// it covers the slowest command you use (flash writes and searches can take ~1 s).
///
/// Ready-made adapters are available behind feature flags:
///
/// - `std`: [`IoTransport`] for any `std::io::Read + std::io::Write` (e.g. `serialport`).
/// - `embedded-io`: [`EmbeddedIoTransport`] for any `embedded_io::Read + embedded_io::Write`.
pub trait Transport {
    type Error;

    /// Transmits all bytes in `data`.
    fn write_all(&mut self, data: &[u8]) -> Result<(), Self::Error>;

    /// Flushes any pending output buffers.
    fn flush(&mut self) -> Result<(), Self::Error>;

    /// Reads incoming bytes into `buf`, returning the number of bytes read
    /// (`0` when the read timed out).
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error>;
}

impl<T: Transport + ?Sized> Transport for &mut T {
    type Error = T::Error;

    fn write_all(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        (**self).write_all(data)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        (**self).flush()
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        (**self).read(buf)
    }
}

/// [`Transport`] adapter for `std::io::Read + std::io::Write` streams (e.g. a `serialport` port).
///
/// Read timeouts (`TimedOut` / `WouldBlock`) and `Interrupted` are mapped to `Ok(0)`.
///
/// ```no_run
/// use grow_r503::{GrowR503, IoTransport};
/// use std::time::Duration;
///
/// let port = serialport::new("/dev/ttyS0", 57600)
///     .timeout(Duration::from_millis(2000))
///     .open()
///     .unwrap();
/// let mut sensor = GrowR503::new(IoTransport::new(port));
/// ```
#[cfg(feature = "std")]
#[derive(Debug)]
pub struct IoTransport<T>(T);

#[cfg(feature = "std")]
impl<T> IoTransport<T> {
    /// Wraps an I/O stream.
    pub fn new(io: T) -> Self {
        Self(io)
    }

    /// Borrows the inner stream.
    pub fn get_ref(&self) -> &T {
        &self.0
    }

    /// Mutably borrows the inner stream.
    pub fn get_mut(&mut self) -> &mut T {
        &mut self.0
    }

    /// Returns the inner stream.
    pub fn into_inner(self) -> T {
        self.0
    }
}

#[cfg(feature = "std")]
impl<T: std::io::Read + std::io::Write> Transport for IoTransport<T> {
    type Error = std::io::Error;

    fn write_all(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        std::io::Write::write_all(&mut self.0, data)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        std::io::Write::flush(&mut self.0)
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        use std::io::ErrorKind;
        match std::io::Read::read(&mut self.0, buf) {
            Ok(n) => Ok(n),
            Err(e)
                if matches!(
                    e.kind(),
                    ErrorKind::TimedOut | ErrorKind::WouldBlock | ErrorKind::Interrupted
                ) =>
            {
                Ok(0)
            }
            Err(e) => Err(e),
        }
    }
}

/// [`Transport`] adapter for `embedded_io::Read + embedded_io::Write` UARTs
/// (as provided by `embedded-hal` 1.0-era HALs such as `esp-hal`, `rp-hal`, `embassy`).
#[cfg(feature = "embedded-io")]
#[derive(Debug)]
pub struct EmbeddedIoTransport<T>(T);

#[cfg(feature = "embedded-io")]
impl<T> EmbeddedIoTransport<T> {
    /// Wraps an `embedded-io` UART.
    pub fn new(uart: T) -> Self {
        Self(uart)
    }

    /// Borrows the inner UART.
    pub fn get_ref(&self) -> &T {
        &self.0
    }

    /// Mutably borrows the inner UART.
    pub fn get_mut(&mut self) -> &mut T {
        &mut self.0
    }

    /// Returns the inner UART.
    pub fn into_inner(self) -> T {
        self.0
    }
}

#[cfg(feature = "embedded-io")]
impl<T: embedded_io::Read + embedded_io::Write> Transport for EmbeddedIoTransport<T> {
    type Error = T::Error;

    fn write_all(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        embedded_io::Write::write_all(&mut self.0, data)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io::Write::flush(&mut self.0)
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io::Read::read(&mut self.0, buf)
    }
}

/// System configuration and status parameters returned by `ReadSysPara` (`0x0F`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SystemParameters {
    /// Status register contents.
    pub status_register: u16,
    /// Hardware system identifier code.
    pub system_id: u16,
    /// Fingerprint library capacity (e.g. 150 or 200).
    pub capacity: u16,
    /// Security threshold level (1 to 5).
    pub security_level: u16,
    /// 32-bit module device address.
    pub device_address: u32,
    /// Data packet chunk length in bytes (32, 64, 128, or 256).
    pub packet_size: u16,
    /// Serial transmission baud rate (multiplier * 9600 bps).
    pub baud_rate: u32,
}

impl SystemParameters {
    /// Parses 16 bytes of parameter data from a `ReadSysPara` response.
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 16 {
            return None;
        }

        let status_register = ((data[0] as u16) << 8) | (data[1] as u16);
        let system_id = ((data[2] as u16) << 8) | (data[3] as u16);
        let capacity = ((data[4] as u16) << 8) | (data[5] as u16);
        let security_level = ((data[6] as u16) << 8) | (data[7] as u16);
        let device_address = ((data[8] as u32) << 24)
            | ((data[9] as u32) << 16)
            | ((data[10] as u32) << 8)
            | (data[11] as u32);
        let packet_code = ((data[12] as u16) << 8) | (data[13] as u16);
        let packet_size = match packet_code {
            0 => 32,
            1 => 64,
            2 => 128,
            3 => 256,
            _ => 128,
        };
        let baud_code = ((data[14] as u32) << 8) | (data[15] as u32);
        let baud_rate = baud_code * 9600;

        Some(Self {
            status_register,
            system_id,
            capacity,
            security_level,
            device_address,
            packet_size,
            baud_rate,
        })
    }
}
