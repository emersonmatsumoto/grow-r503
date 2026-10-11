# grow-r503

[![Crates.io](https://img.shields.io/crates/v/grow-r503.svg)](https://crates.io/crates/grow-r503)
[![Documentation](https://docs.rs/grow-r503/badge.svg)](https://docs.rs/grow-r503)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)

A platform-agnostic, zero-allocation Rust driver for the **Grow R503** capacitive biometric fingerprint sensor with complete circular Aura RGB LED ring support.

Compatible with bare-metal embedded targets (`no_std` on microcontrollers like ESP32, STM32, RP2040) as well as Linux single-board computers (Raspberry Pi, Orange Pi, BeagleBone).

---

## Features

- **Full R503 Protocol Implementation:**
  - 1:1 Biometric matching (`match_templates`)
  - 1:N Database searching (`search`)
  - Dual-sample feature fusion (`create_model`)
  - Flash template storage, loading, and deletion
  - Index table reading (retrieve bitmaps of occupied slots)
  - System parameter and capacity inspection (`read_system_parameters`)
  - Hardware random number generation (`get_random_number`)
  - DSP auto-enrollment with step tracking (`auto_enroll`, `auto_enroll_with`)
- **Aura RGB LED Ring:**
  - Support for all hardware animation modes: `Breathing`, `Flashing`, `AlwaysOn`, `AlwaysOff`, `GradualOn`, `GradualOff`.
  - 7 hardware colors: Red, Blue, Purple, Green, Yellow, Cyan, White.
- **Zero Heap Allocation (`no_std` compatible):**
  - Uses fixed stack buffers for packet construction and framing.
  - Zero dynamic memory allocations (`Vec` or `String` not required).
- **Serial Resilience:**
  - Resynchronizes to `0xEF 0x01` headers after transmission line noise.
  - Automatically recovers from dropped synchronization bytes caused by Linux TTY line disciplines across all device addresses.

---

## Hardware Pinout (Grow R503)

| Wire Color | Pin Name | Description | Recommended SBC / MCU Connection |
|:---|:---|:---|:---|
| **Red** | VCC | Sensor power supply | 3.3V DC |
| **Black** | GND | Ground | GND |
| **Yellow** | TX | Sensor Serial Output (TTL) | MCU / SBC UART RX |
| **Green / Brown** | RX | Sensor Serial Input (TTL) | MCU / SBC UART TX |
| **White** | Touch VCC | Touch detection circuit power | 3.3V DC (or 3.3V power rail) |
| **Blue** | Touch Out | Finger presence interrupt signal | Active High GPIO Input |

*Default serial parameters: **57600 baud**, 8 data bits, 1 stop bit, no parity.*

---

## Usage

### In a Standard Linux Environment (`std`)

Add to `Cargo.toml`:
```toml
[dependencies]
grow-r503 = "0.1.5"
serialport = "4.10"
```

```rust
use grow_r503::{GrowR503, IoTransport};
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port = serialport::new("/dev/ttyS0", 57600)
        .timeout(Duration::from_millis(2000))
        .open()?;

    let mut sensor = GrowR503::new(IoTransport::new(port));

    // Initial password handshake (default: 0x00000000)
    sensor.verify_password([0x00, 0x00, 0x00, 0x00])?;

    // Set Aura LED ring to gentle breathing blue glow
    sensor.led_standby()?;

    // Read total number of stored fingerprints
    let count = sensor.get_template_count()?;
    println!("Enrolled templates in flash: {}", count);

    // Check system capacity and parameters
    let params = sensor.read_system_parameters()?;
    println!("Sensor capacity: {} fingerprints", params.capacity);
    println!("Baud rate: {} bps", params.baud_rate);

    Ok(())
}
```

---

### In an Embedded Environment with `embedded-io`

Enable the `embedded-io` feature:
```toml
[dependencies]
grow-r503 = { version = "0.1.5", default-features = false, features = ["embedded-io"] }
```

Wrap your HAL UART peripheral implementing `embedded_io::Read` and `embedded_io::Write`:
```rust,ignore
use grow_r503::{EmbeddedIoTransport, GrowR503};

let transport = EmbeddedIoTransport::new(uart);
let mut sensor = GrowR503::new(transport);
```

---

### In a Bare-Metal Embedded Environment (`no_std` custom transport)

Disable default features in `Cargo.toml`:
```toml
[dependencies]
grow-r503 = { version = "0.1.5", default-features = false }
```

Implement the `Transport` trait for your UART peripheral:
```rust
use grow_r503::{GrowR503, Transport};

struct MyEmbeddedUart(/* your peripheral */);

impl Transport for MyEmbeddedUart {
    type Error = MyUartError;

    fn write_all(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        // write bytes to hardware UART TX FIFO
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // wait for transmit complete
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // read received bytes into buf, return Ok(0) on timeout
        Ok(bytes_read)
    }
}
```

---

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT License ([LICENSE-MIT](LICENSE-MIT))

at your option.

