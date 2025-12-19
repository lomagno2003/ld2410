# LD2410 Rust Driver

A `no_std` compatible Rust driver for the HLK-LD2410 human presence radar sensor. This library provides a robust interface for communicating with the LD2410 over UART using the vendor's binary framed protocol.

## Features

- **No-std compatible** – works in embedded environments without the standard library
- **Robust frame parsing** – handles continuous sensor streams with proper header/footer validation
- **Normal and engineering modes** – read presence data or switch to engineering mode for detailed diagnostics
- **Configuration support** – enter/exit config mode, read firmware, restart, and more
- **Flexible distance parsing** – handles firmware variations in distance field encoding

## Protocol Overview

The LD2410 communicates via UART at 256000 baud (8N1) using a binary framed protocol:

### Sensor Reports (radar → host)
```
[Header: F4 F3 F2 F1][Length: u16 LE][Frame Data][Footer: F8 F7 F6 F5]
```

Frame data contains:
- Type byte (0x02 for normal mode, 0x01 for engineering)
- Inner marker (0xAA)
- Payload
- Inner tail (0x55) and check (0x00)

### Command/ACK Frames (host ↔ radar)
```
[Header: FD FC FB FA][Length: u16 LE][Command Word: u16 LE][Payload][Footer: 04 03 02 01]
```

## Usage

### Basic Presence Detection

```rust
use ld2410::LD2410;

// Create sensor with UART peripheral
let mut sensor = LD2410::new(uart);

// Read presence data continuously
loop {
    match sensor.read_presence() {
        Ok(data) => {
            println!("Presence: {}", data.presence());
            println!("Moving: {}cm (energy: {})", 
                data.moving_distance_cm, 
                data.moving_energy);
            println!("Still: {}cm (energy: {})", 
                data.still_distance_cm, 
                data.still_energy);
        }
        Err(e) => println!("Error: {:?}", e),
    }
}
```

### Configuration

```rust
// Enter configuration mode
sensor.enter_config_mode()?;

// Read firmware version
let fw = sensor.read_firmware_raw()?;

// Exit configuration mode
sensor.exit_config_mode()?;
```

### Engineering Mode

```rust
// Enable engineering mode for detailed diagnostics
sensor.enable_engineering_mode()?;

// Read engineering reports (type 0x01)
// ... handle engineering data ...

// Return to normal mode
sensor.disable_engineering_mode()?;
```

## Data Structures

### PresenceData
Contains the parsed presence detection report:
- `target_state` – NoTarget, Moving, Stationary, or Both
- `moving_distance_cm` – distance to moving target
- `moving_energy` – energy level of moving target (0-255)
- `still_distance_cm` – distance to stationary target
- `still_energy` – energy level of stationary target (0-255)
- `detection_distance_cm` – overall detection distance

### Error Types
- `Read(E)` – UART read error
- `Write(E)` – UART write error
- `InvalidFrame` – malformed frame structure
- `UnknownReportType` – unsupported report type
- `InvalidReport` – report content too short or inconsistent
- `BufferOverflow` – received frame exceeds buffer size

## Requirements

- UART peripheral implementing `embedded-io` traits (`Read` + `Write`)
- 256000 baud, 8N1 configuration
- Sufficient buffer space (256 bytes internal)

## Example

See `../hello-world-ld2410/` for a complete ESP32 example using esp-hal and embassy.

## License

MIT