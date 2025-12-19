#![no_std]

//! LD2410 Rust Driver
//!
//! A no_std compatible driver for the HLK-LD2410 human presence radar sensor.
//! Communicates over UART at 256000 baud using a binary framed protocol.

/// Error types for LD2410 driver operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// UART communication error
    Uart(E),
    /// Frame reception timeout
    Timeout,
    /// CRC/checksum mismatch
    InvalidChecksum,
    /// Malformed frame structure
    InvalidFrame,
}

/// Presence detection data from the LD2410 sensor
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PresenceData {
    /// Whether any human presence is detected
    pub presence: bool,
    /// Distance to detected motion in centimeters (0 if no motion detected)
    pub motion_distance_cm: u16,
    /// Distance to detected static presence in centimeters (0 if no static presence)
    pub static_distance_cm: u16,
    /// Motion energy level (0-255, intensity of movement)
    pub motion_energy: u8,
    /// Static energy level (0-255, intensity of stationary presence)
    pub static_energy: u8,
}

/// Sensor configuration parameters
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorConfig {
    /// Maximum detection distance in centimeters
    pub max_distance_cm: u16,
    /// Motion sensitivity (0-100)
    pub motion_sensitivity: u8,
    /// Static sensitivity (0-100)
    pub static_sensitivity: u8,
}

/// Frame protocol constants
pub mod frame {
    /// Frame header bytes (always 0xFD, 0xFC, 0xFB, 0xFA)
    pub const HEADER: [u8; 4] = [0xFD, 0xFC, 0xFB, 0xFA];
    /// Maximum data payload size in a frame
    pub const MAX_DATA_SIZE: usize = 128;
    /// Total maximum frame size (header + length + command + data + checksum)
    pub const MAX_FRAME_SIZE: usize = 4 + 2 + 1 + MAX_DATA_SIZE + 2;

    /// Frame structure for command/response communication
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct Frame {
        /// Command/response type
        pub command: u8,
        /// Variable length data payload
        pub data: [u8; MAX_DATA_SIZE],
        /// Actual length of data in use
        pub data_len: usize,
    }

    impl Frame {
        /// Create a new frame with the given command and data
        pub fn new(command: u8, data: &[u8]) -> Result<Self, crate::Error<()>> {
            if data.len() > MAX_DATA_SIZE {
                return Err(crate::Error::InvalidFrame);
            }

            let mut frame = Frame {
                command,
                data: [0u8; MAX_DATA_SIZE],
                data_len: data.len(),
            };

            frame.data[..data.len()].copy_from_slice(data);
            Ok(frame)
        }

        /// Serialize frame to bytes with header and checksum
        pub fn serialize(&self) -> [u8; MAX_FRAME_SIZE] {
            let mut buffer = [0u8; MAX_FRAME_SIZE];
            let mut pos = 0;

            // Header
            buffer[pos..pos + 4].copy_from_slice(&HEADER);
            pos += 4;

            // Length (command + data, little-endian)
            let length = (1 + self.data_len) as u16;
            buffer[pos] = (length & 0xFF) as u8;
            buffer[pos + 1] = ((length >> 8) & 0xFF) as u8;
            pos += 2;

            // Command
            buffer[pos] = self.command;
            pos += 1;

            // Data
            buffer[pos..pos + self.data_len].copy_from_slice(&self.data[..self.data_len]);
            pos += self.data_len;

            // Calculate checksum over header + length + command + data
            let checksum = crate::frame::calculate_checksum(&buffer[..pos]);
            buffer[pos] = (checksum & 0xFF) as u8;
            buffer[pos + 1] = ((checksum >> 8) & 0xFF) as u8;

            buffer
        }

        /// Get the serialized frame length (header + length + command + data + checksum)
        pub fn serialized_len(&self) -> usize {
            4 + 2 + 1 + self.data_len + 2
        }
    }

    /// Calculate CRC16 (CCITT) checksum
    pub fn calculate_checksum(data: &[u8]) -> u16 {
        let mut crc: u32 = 0;

        for &byte in data {
            crc ^= (byte as u32) << 8;
            for _ in 0..8 {
                crc <<= 1;
                if (crc & 0x10000) != 0 {
                    crc ^= 0x1021;
                }
            }
        }

        (crc & 0xFFFF) as u16
    }

    /// Validate frame checksum
    pub fn validate_checksum(data: &[u8], checksum: u16) -> bool {
        calculate_checksum(data) == checksum
    }

    /// Parse a frame from raw bytes
    pub fn parse_frame(buffer: &[u8]) -> Result<Frame, crate::Error<()>> {
        // Minimum frame size: header(4) + length(2) + command(1) + checksum(2)
        if buffer.len() < 9 {
            return Err(crate::Error::InvalidFrame);
        }

        // Validate header
        if &buffer[0..4] != &HEADER {
            return Err(crate::Error::InvalidFrame);
        }

        // Parse length (little-endian)
        let length = ((buffer[5] as u16) << 8) | (buffer[4] as u16);
        let length = length as usize;

        // Validate frame size
        let expected_size = 4 + 2 + length + 2;
        if buffer.len() < expected_size {
            return Err(crate::Error::InvalidFrame);
        }

        // Extract checksum (little-endian)
        let checksum_pos = 4 + 2 + length;
        let checksum = ((buffer[checksum_pos + 1] as u16) << 8) | (buffer[checksum_pos] as u16);

        // Validate checksum
        if !validate_checksum(&buffer[..checksum_pos], checksum) {
            return Err(crate::Error::InvalidChecksum);
        }

        // Extract command
        let command = buffer[6];

        // Extract data
        let data_len = length.saturating_sub(1);
        if data_len > MAX_DATA_SIZE {
            return Err(crate::Error::InvalidFrame);
        }

        let mut data = [0u8; MAX_DATA_SIZE];
        if data_len > 0 {
            data[..data_len].copy_from_slice(&buffer[7..7 + data_len]);
        }

        Ok(Frame {
            command,
            data,
            data_len,
        })
    }
}

/// LD2410 driver for UART communication
///
/// Generic over UART peripheral and delay provider for no_std compatibility.
pub struct LD2410<UART, D> {
    uart: UART,
    delay: D,
    timeout_ms: u32,
}

impl<UART, D> LD2410<UART, D> {
    /// Create a new LD2410 driver with default timeout (1000ms)
    pub fn new(uart: UART, delay: D) -> Self {
        Self {
            uart,
            delay,
            timeout_ms: 1000,
        }
    }

    /// Create a new LD2410 driver with custom timeout
    pub fn with_timeout(uart: UART, delay: D, timeout_ms: u32) -> Self {
        Self {
            uart,
            delay,
            timeout_ms,
        }
    }
}

/// UART traits for embedded-hal compatibility
pub mod uart_traits {
    /// Trait for writing bytes over UART
    pub trait Write {
        /// Error type for UART write operations
        type Error;

        /// Write a single byte
        fn write(&mut self, byte: u8) -> Result<(), Self::Error>;

        /// Write multiple bytes
        fn write_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
            for &byte in buffer {
                self.write(byte)?;
            }
            Ok(())
        }
    }

    /// Trait for reading bytes from UART
    pub trait Read {
        /// Error type for UART read operations
        type Error;

        /// Read a single byte (non-blocking)
        fn read(&mut self) -> Result<Option<u8>, Self::Error>;
    }

    /// Trait for delay operations
    pub trait DelayMs {
        /// Delay for the specified number of milliseconds
        fn delay_ms(&mut self, ms: u32);
    }
}

/// LD2410 driver that accepts separate TX and RX types
///
/// This variant allows using split UART types directly without a wrapper.
pub struct LD2410Split<TX, RX, D> {
    tx: TX,
    rx: RX,
    delay: D,
    timeout_ms: u32,
}

impl<TX, RX, D> LD2410Split<TX, RX, D> {
    /// Create a new LD2410 driver with default timeout (1000ms)
    pub fn new(tx: TX, rx: RX, delay: D) -> Self {
        Self {
            tx,
            rx,
            delay,
            timeout_ms: 1000,
        }
    }

    /// Create a new LD2410 driver with custom timeout
    pub fn with_timeout(tx: TX, rx: RX, delay: D, timeout_ms: u32) -> Self {
        Self {
            tx,
            rx,
            delay,
            timeout_ms,
        }
    }
}

impl<UART, D, E> LD2410<UART, D>
where
    UART: uart_traits::Write<Error = E> + uart_traits::Read<Error = E>,
    D: uart_traits::DelayMs,
    E: core::fmt::Debug,
{
    /// Send a frame over UART
    ///
    /// Serializes the frame and transmits all bytes over UART.
    /// Returns an error if any byte transmission fails.
    ///
    /// # Requirements
    /// - Requirement 1.1: UART configured to 256000 baud, 8 data bits, no parity, 1 stop bit
    /// - Requirement 1.2: Frame formatted with 4-byte header, command bytes, and 2-byte checksum
    pub fn send_frame(&mut self, frame: &frame::Frame) -> Result<(), Error<E>> {
        let serialized = frame.serialize();
        let frame_len = frame.serialized_len();

        self.uart
            .write_all(&serialized[..frame_len])
            .map_err(Error::Uart)
    }

    /// Read presence detection data from the LD2410 sensor
    ///
    /// Sends a read presence command and parses the sensor response into PresenceData.
    /// Extracts motion distance, static distance, motion energy, and static energy.
    ///
    /// # Returns
    /// - `Ok(PresenceData)` with current sensor readings
    /// - `Err(Error::Timeout)` if no response received within timeout
    /// - `Err(Error::InvalidChecksum)` if response frame checksum is invalid
    /// - `Err(Error::InvalidFrame)` if response frame is malformed
    /// - `Err(Error::Uart(E))` if UART communication fails
    ///
    /// # Requirements
    /// - Requirement 2.1: Return presence state (present or not present)
    /// - Requirement 2.2: Return distance to detected motion in centimeters
    /// - Requirement 2.3: Return distance to detected static presence in centimeters
    /// - Requirement 2.4: Return motion energy level (0-255)
    /// - Requirement 2.5: Return static energy level (0-255)
    pub fn read_presence(&mut self) -> Result<PresenceData, Error<E>> {
        // Send read presence command (0x01)
        let cmd_frame = frame::Frame::new(0x01, &[]).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        // Receive response frame
        let response = self.receive_frame()?;

        // Parse presence data from response
        // Expected response format:
        // Byte 0: Presence state (0x00 = no presence, 0x01 = presence detected)
        // Bytes 1-2: Motion distance (little-endian u16)
        // Bytes 3-4: Static distance (little-endian u16)
        // Byte 5: Motion energy (u8)
        // Byte 6: Static energy (u8)
        
        if response.data_len < 7 {
            return Err(Error::InvalidFrame);
        }

        let presence = response.data[0] != 0x00;
        let motion_distance_cm =
            ((response.data[2] as u16) << 8) | (response.data[1] as u16);
        let static_distance_cm =
            ((response.data[4] as u16) << 8) | (response.data[3] as u16);
        let motion_energy = response.data[5];
        let static_energy = response.data[6];

        Ok(PresenceData {
            presence,
            motion_distance_cm,
            static_distance_cm,
            motion_energy,
            static_energy,
        })
    }

    /// Receive a frame from UART with timeout
    ///
    /// Reads bytes from UART until a complete frame is received or timeout occurs.
    /// Validates the frame header and checksum before returning.
    ///
    /// # Returns
    /// - `Ok(Frame)` if a valid frame is received
    /// - `Err(Error::Timeout)` if no complete frame is received within timeout_ms
    /// - `Err(Error::InvalidChecksum)` if frame checksum is invalid
    /// - `Err(Error::InvalidFrame)` if frame structure is invalid
    /// - `Err(Error::Uart(E))` if UART read fails
    ///
    /// # Requirements
    /// - Requirement 1.3: Validate frame header and checksum before processing
    /// - Requirement 1.4: Reject frames with invalid checksums
    /// - Requirement 1.5: Handle incomplete or malformed frames gracefully
    /// - Requirement 4.2: Return timeout error if frame reception times out
    pub fn receive_frame(&mut self) -> Result<frame::Frame, Error<E>> {
        let mut buffer = [0u8; frame::MAX_FRAME_SIZE];
        let mut pos = 0;
        let mut elapsed_ms = 0u32;
        let poll_interval_ms = 1u32;

        loop {
            // Check timeout
            if elapsed_ms >= self.timeout_ms {
                return Err(Error::Timeout);
            }

            // Try to read a byte
            match self.uart.read().map_err(Error::Uart)? {
                Some(byte) => {
                    buffer[pos] = byte;
                    pos += 1;

                    // Check if we have a complete frame
                    if pos >= 9 {
                        // Minimum frame size: header(4) + length(2) + command(1) + checksum(2)
                        // Try to parse what we have so far
                        match frame::parse_frame(&buffer[..pos]) {
                            Ok(parsed_frame) => {
                                return Ok(parsed_frame);
                            }
                            Err(Error::InvalidChecksum) => {
                                // Checksum error - frame is complete but invalid
                                return Err(Error::InvalidChecksum);
                            }
                            Err(Error::InvalidFrame) => {
                                // Check if we have an invalid header (first 4 bytes don't match)
                                if pos >= 4 && &buffer[0..4] != &frame::HEADER {
                                    // Invalid header detected - this is definitely wrong
                                    return Err(Error::InvalidFrame);
                                }

                                // Could be incomplete or malformed
                                // If we've read too much without finding a valid frame, it's malformed
                                if pos >= frame::MAX_FRAME_SIZE {
                                    return Err(Error::InvalidFrame);
                                }
                                // Otherwise, keep reading
                            }
                            Err(Error::Timeout) => {
                                // This shouldn't happen from parse_frame, but handle it
                                return Err(Error::Timeout);
                            }
                            Err(Error::Uart(_)) => {
                                // This shouldn't happen from parse_frame
                                return Err(Error::InvalidFrame);
                            }
                        }
                    }

                    // Reset elapsed time on successful byte read
                    elapsed_ms = 0;
                }
                None => {
                    // No byte available, delay and increment timeout counter
                    self.delay.delay_ms(poll_interval_ms);
                    elapsed_ms += poll_interval_ms;
                }
            }
        }
    }

    /// Set the maximum detection distance
    ///
    /// Sends a configuration command to set the maximum distance the sensor will detect.
    /// The sensor will not report presence beyond this distance.
    ///
    /// # Arguments
    /// * `distance_cm` - Maximum detection distance in centimeters
    ///
    /// # Returns
    /// - `Ok(())` if configuration was successfully sent and acknowledged
    /// - `Err(Error::Timeout)` if no response received within timeout
    /// - `Err(Error::InvalidChecksum)` if response frame checksum is invalid
    /// - `Err(Error::InvalidFrame)` if response frame is malformed
    /// - `Err(Error::Uart(E))` if UART communication fails
    ///
    /// # Requirements
    /// - Requirement 3.1: Send the configuration command and validate the response
    pub fn set_max_distance(&mut self, distance_cm: u16) -> Result<(), Error<E>> {
        // Command 0x03: Set maximum distance
        // Data: 2 bytes for distance (little-endian)
        let data = [
            (distance_cm & 0xFF) as u8,
            ((distance_cm >> 8) & 0xFF) as u8,
        ];

        let cmd_frame = frame::Frame::new(0x03, &data).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        // Receive and validate response
        let _response = self.receive_frame()?;

        Ok(())
    }

    /// Set the motion sensitivity
    ///
    /// Sends a configuration command to set the motion detection sensitivity.
    /// Higher values increase sensitivity to motion.
    ///
    /// # Arguments
    /// * `sensitivity` - Motion sensitivity value (0-100)
    ///
    /// # Returns
    /// - `Ok(())` if configuration was successfully sent and acknowledged
    /// - `Err(Error::Timeout)` if no response received within timeout
    /// - `Err(Error::InvalidChecksum)` if response frame checksum is invalid
    /// - `Err(Error::InvalidFrame)` if response frame is malformed or sensitivity is invalid
    /// - `Err(Error::Uart(E))` if UART communication fails
    ///
    /// # Requirements
    /// - Requirement 3.2: Accept values in the valid range and send the configuration command
    /// - Requirement 3.4: Reject invalid parameter values and return an error
    pub fn set_motion_sensitivity(&mut self, sensitivity: u8) -> Result<(), Error<E>> {
        // Validate parameter range
        if sensitivity > 100 {
            return Err(Error::InvalidFrame);
        }

        // Command 0x04: Set motion sensitivity
        // Data: 1 byte for sensitivity
        let data = [sensitivity];

        let cmd_frame = frame::Frame::new(0x04, &data).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        // Receive and validate response
        let _response = self.receive_frame()?;

        Ok(())
    }

    /// Set the static sensitivity
    ///
    /// Sends a configuration command to set the static presence detection sensitivity.
    /// Higher values increase sensitivity to stationary presence.
    ///
    /// # Arguments
    /// * `sensitivity` - Static sensitivity value (0-100)
    ///
    /// # Returns
    /// - `Ok(())` if configuration was successfully sent and acknowledged
    /// - `Err(Error::Timeout)` if no response received within timeout
    /// - `Err(Error::InvalidChecksum)` if response frame checksum is invalid
    /// - `Err(Error::InvalidFrame)` if response frame is malformed or sensitivity is invalid
    /// - `Err(Error::Uart(E))` if UART communication fails
    ///
    /// # Requirements
    /// - Requirement 3.3: Accept values in the valid range and send the configuration command
    /// - Requirement 3.4: Reject invalid parameter values and return an error
    pub fn set_static_sensitivity(&mut self, sensitivity: u8) -> Result<(), Error<E>> {
        // Validate parameter range
        if sensitivity > 100 {
            return Err(Error::InvalidFrame);
        }

        // Command 0x05: Set static sensitivity
        // Data: 1 byte for sensitivity
        let data = [sensitivity];

        let cmd_frame = frame::Frame::new(0x05, &data).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        // Receive and validate response
        let _response = self.receive_frame()?;

        Ok(())
    }

    /// Save configuration to non-volatile memory
    ///
    /// Sends a command to persist the current configuration settings to the sensor's
    /// non-volatile memory. Configuration changes will be retained after power cycles.
    ///
    /// # Returns
    /// - `Ok(())` if save command was successfully sent and acknowledged
    /// - `Err(Error::Timeout)` if no response received within timeout
    /// - `Err(Error::InvalidChecksum)` if response frame checksum is invalid
    /// - `Err(Error::InvalidFrame)` if response frame is malformed
    /// - `Err(Error::Uart(E))` if UART communication fails
    ///
    /// # Requirements
    /// - Requirement 3.5: Persist settings to the sensor's non-volatile memory
    pub fn save_config(&mut self) -> Result<(), Error<E>> {
        // Command 0x06: Save configuration
        // No data payload
        let cmd_frame = frame::Frame::new(0x06, &[]).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        // Receive and validate response
        let _response = self.receive_frame()?;

        Ok(())
    }

    /// Reset the sensor to default settings
    ///
    /// Sends a reset command to restore the sensor to its factory default configuration.
    /// All custom settings will be lost after reset.
    ///
    /// # Returns
    /// - `Ok(())` if reset command was successfully sent and acknowledged
    /// - `Err(Error::Timeout)` if no response received within timeout
    /// - `Err(Error::InvalidChecksum)` if response frame checksum is invalid
    /// - `Err(Error::InvalidFrame)` if response frame is malformed
    /// - `Err(Error::Uart(E))` if UART communication fails
    ///
    /// # Requirements
    /// - Requirement 6.1: Create `reset()` method
    pub fn reset(&mut self) -> Result<(), Error<E>> {
        // Command 0x07: Reset to defaults
        // No data payload
        let cmd_frame = frame::Frame::new(0x07, &[]).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        // Receive and validate response
        let _response = self.receive_frame()?;

        Ok(())
    }

    /// Get the firmware version of the LD2410 sensor
    ///
    /// Sends a command to retrieve the firmware version information from the sensor.
    /// Returns the major, minor, and patch version numbers.
    ///
    /// # Returns
    /// - `Ok((major, minor, patch))` with firmware version numbers
    /// - `Err(Error::Timeout)` if no response received within timeout
    /// - `Err(Error::InvalidChecksum)` if response frame checksum is invalid
    /// - `Err(Error::InvalidFrame)` if response frame is malformed or missing version data
    /// - `Err(Error::Uart(E))` if UART communication fails
    ///
    /// # Requirements
    /// - Requirement 6.1: Create `get_version()` method
    pub fn get_version(&mut self) -> Result<(u8, u8, u8), Error<E>> {
        // Command 0x08: Get firmware version
        // No data payload
        let cmd_frame = frame::Frame::new(0x08, &[]).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        // Receive response frame
        let response = self.receive_frame()?;

        // Parse version data from response
        // Expected response format:
        // Bytes 0-2: Major, Minor, Patch version numbers
        if response.data_len < 3 {
            return Err(Error::InvalidFrame);
        }

        let major = response.data[0];
        let minor = response.data[1];
        let patch = response.data[2];

        Ok((major, minor, patch))
    }

    /// Read the current sensor configuration
    ///
    /// Sends a command to retrieve the current configuration settings from the sensor.
    /// Returns the maximum detection distance, motion sensitivity, and static sensitivity.
    ///
    /// # Returns
    /// - `Ok(SensorConfig)` with current configuration parameters
    /// - `Err(Error::Timeout)` if no response received within timeout
    /// - `Err(Error::InvalidChecksum)` if response frame checksum is invalid
    /// - `Err(Error::InvalidFrame)` if response frame is malformed or missing config data
    /// - `Err(Error::Uart(E))` if UART communication fails
    ///
    /// # Requirements
    /// - Requirement 6.1: Create `read_config()` method
    pub fn read_config(&mut self) -> Result<SensorConfig, Error<E>> {
        // Command 0x02: Read configuration
        // No data payload
        let cmd_frame = frame::Frame::new(0x02, &[]).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        // Receive response frame
        let response = self.receive_frame()?;

        // Parse configuration data from response
        // Expected response format:
        // Bytes 0-1: Maximum distance (little-endian u16)
        // Byte 2: Motion sensitivity (u8)
        // Byte 3: Static sensitivity (u8)
        if response.data_len < 4 {
            return Err(Error::InvalidFrame);
        }

        let max_distance_cm = ((response.data[1] as u16) << 8) | (response.data[0] as u16);
        let motion_sensitivity = response.data[2];
        let static_sensitivity = response.data[3];

        Ok(SensorConfig {
            max_distance_cm,
            motion_sensitivity,
            static_sensitivity,
        })
    }
}

impl<TX, RX, D, E> LD2410Split<TX, RX, D>
where
    TX: uart_traits::Write<Error = E>,
    RX: uart_traits::Read<Error = E>,
    D: uart_traits::DelayMs,
    E: core::fmt::Debug,
{
    /// Send a frame over UART
    pub fn send_frame(&mut self, frame: &frame::Frame) -> Result<(), Error<E>> {
        let serialized = frame.serialize();
        let frame_len = frame.serialized_len();

        self.tx
            .write_all(&serialized[..frame_len])
            .map_err(Error::Uart)
    }

    /// Read presence detection data from the LD2410 sensor
    pub fn read_presence(&mut self) -> Result<PresenceData, Error<E>> {
        let cmd_frame = frame::Frame::new(0x01, &[]).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        let response = self.receive_frame()?;

        if response.data_len < 7 {
            return Err(Error::InvalidFrame);
        }

        let presence = response.data[0] != 0x00;
        let motion_distance_cm =
            ((response.data[2] as u16) << 8) | (response.data[1] as u16);
        let static_distance_cm =
            ((response.data[4] as u16) << 8) | (response.data[3] as u16);
        let motion_energy = response.data[5];
        let static_energy = response.data[6];

        Ok(PresenceData {
            presence,
            motion_distance_cm,
            static_distance_cm,
            motion_energy,
            static_energy,
        })
    }

    /// Receive a frame from UART with timeout
    pub fn receive_frame(&mut self) -> Result<frame::Frame, Error<E>> {
        let mut buffer = [0u8; frame::MAX_FRAME_SIZE];
        let mut pos = 0;
        let mut elapsed_ms = 0u32;
        let poll_interval_ms = 1u32;

        loop {
            if elapsed_ms >= self.timeout_ms {
                return Err(Error::Timeout);
            }

            match self.rx.read().map_err(Error::Uart)? {
                Some(byte) => {
                    buffer[pos] = byte;
                    pos += 1;

                    if pos >= 9 {
                        match frame::parse_frame(&buffer[..pos]) {
                            Ok(parsed_frame) => {
                                return Ok(parsed_frame);
                            }
                            Err(Error::InvalidChecksum) => {
                                return Err(Error::InvalidChecksum);
                            }
                            Err(Error::InvalidFrame) => {
                                if pos >= 4 && &buffer[0..4] != &frame::HEADER {
                                    return Err(Error::InvalidFrame);
                                }

                                if pos >= frame::MAX_FRAME_SIZE {
                                    return Err(Error::InvalidFrame);
                                }
                            }
                            Err(Error::Timeout) => {
                                return Err(Error::Timeout);
                            }
                            Err(Error::Uart(_)) => {
                                return Err(Error::InvalidFrame);
                            }
                        }
                    }

                    elapsed_ms = 0;
                }
                None => {
                    self.delay.delay_ms(poll_interval_ms);
                    elapsed_ms += poll_interval_ms;
                }
            }
        }
    }

    /// Set the maximum detection distance
    pub fn set_max_distance(&mut self, distance_cm: u16) -> Result<(), Error<E>> {
        let data = [
            (distance_cm & 0xFF) as u8,
            ((distance_cm >> 8) & 0xFF) as u8,
        ];

        let cmd_frame = frame::Frame::new(0x03, &data).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        let _response = self.receive_frame()?;

        Ok(())
    }

    /// Set the motion sensitivity
    pub fn set_motion_sensitivity(&mut self, sensitivity: u8) -> Result<(), Error<E>> {
        if sensitivity > 100 {
            return Err(Error::InvalidFrame);
        }

        let data = [sensitivity];

        let cmd_frame = frame::Frame::new(0x04, &data).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        let _response = self.receive_frame()?;

        Ok(())
    }

    /// Set the static sensitivity
    pub fn set_static_sensitivity(&mut self, sensitivity: u8) -> Result<(), Error<E>> {
        if sensitivity > 100 {
            return Err(Error::InvalidFrame);
        }

        let data = [sensitivity];

        let cmd_frame = frame::Frame::new(0x05, &data).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        let _response = self.receive_frame()?;

        Ok(())
    }

    /// Save configuration to non-volatile memory
    pub fn save_config(&mut self) -> Result<(), Error<E>> {
        let cmd_frame = frame::Frame::new(0x06, &[]).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        let _response = self.receive_frame()?;

        Ok(())
    }

    /// Reset the sensor to default settings
    pub fn reset(&mut self) -> Result<(), Error<E>> {
        let cmd_frame = frame::Frame::new(0x07, &[]).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        let _response = self.receive_frame()?;

        Ok(())
    }

    /// Get the firmware version of the LD2410 sensor
    pub fn get_version(&mut self) -> Result<(u8, u8, u8), Error<E>> {
        let cmd_frame = frame::Frame::new(0x08, &[]).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        let response = self.receive_frame()?;

        if response.data_len < 3 {
            return Err(Error::InvalidFrame);
        }

        let major = response.data[0];
        let minor = response.data[1];
        let patch = response.data[2];

        Ok((major, minor, patch))
    }

    /// Read the current sensor configuration
    pub fn read_config(&mut self) -> Result<SensorConfig, Error<E>> {
        let cmd_frame = frame::Frame::new(0x02, &[]).map_err(|_| Error::InvalidFrame)?;
        self.send_frame(&cmd_frame)?;

        let response = self.receive_frame()?;

        if response.data_len < 4 {
            return Err(Error::InvalidFrame);
        }

        let max_distance_cm = ((response.data[1] as u16) << 8) | (response.data[0] as u16);
        let motion_sensitivity = response.data[2];
        let static_sensitivity = response.data[3];

        Ok(SensorConfig {
            max_distance_cm,
            motion_sensitivity,
            static_sensitivity,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::frame::*;

    #[test]
    fn test_presence_data_creation() {
        let data = PresenceData {
            presence: true,
            motion_distance_cm: 100,
            static_distance_cm: 150,
            motion_energy: 50,
            static_energy: 30,
        };

        assert_eq!(data.presence, true);
        assert_eq!(data.motion_distance_cm, 100);
        assert_eq!(data.static_distance_cm, 150);
        assert_eq!(data.motion_energy, 50);
        assert_eq!(data.static_energy, 30);
    }

    #[test]
    fn test_sensor_config_creation() {
        let config = SensorConfig {
            max_distance_cm: 600,
            motion_sensitivity: 50,
            static_sensitivity: 40,
        };

        assert_eq!(config.max_distance_cm, 600);
        assert_eq!(config.motion_sensitivity, 50);
        assert_eq!(config.static_sensitivity, 40);
    }

    #[test]
    fn test_error_types() {
        let _err: Error<&str> = Error::Timeout;
        let _err: Error<&str> = Error::InvalidChecksum;
        let _err: Error<&str> = Error::InvalidFrame;
        let _err: Error<&str> = Error::Uart("uart error");
    }

    // Frame protocol tests
    #[test]
    fn test_frame_header_consistency() {
        // Property 7: Frame Header Consistency
        // For any frame generated by the driver, the header must always be exactly
        // [0xFD, 0xFC, 0xFB, 0xFA] in that order.
        let frame = Frame::new(0x01, &[0x10, 0x20, 0x30]).unwrap();
        let serialized = frame.serialize();

        assert_eq!(&serialized[0..4], &HEADER);
        assert_eq!(serialized[0], 0xFD);
        assert_eq!(serialized[1], 0xFC);
        assert_eq!(serialized[2], 0xFB);
        assert_eq!(serialized[3], 0xFA);
    }

    #[test]
    fn test_frame_serialization_with_empty_data() {
        let frame = Frame::new(0x01, &[]).unwrap();
        let serialized = frame.serialize();

        // Header (4) + Length (2) + Command (1) + Checksum (2) = 9 bytes minimum
        assert!(serialized.len() >= 9);
        assert_eq!(&serialized[0..4], &HEADER);
        assert_eq!(serialized[6], 0x01); // Command
    }

    #[test]
    fn test_frame_serialization_with_data() {
        let data = [0x10, 0x20, 0x30, 0x40];
        let frame = Frame::new(0x02, &data).unwrap();
        let serialized = frame.serialize();

        assert_eq!(&serialized[0..4], &HEADER);
        assert_eq!(serialized[6], 0x02); // Command
        assert_eq!(&serialized[7..11], &data); // Data
    }

    #[test]
    fn test_frame_length_encoding() {
        let data = [0x10, 0x20];
        let frame = Frame::new(0x03, &data).unwrap();
        let serialized = frame.serialize();

        // Length should be command (1) + data (2) = 3
        let length = ((serialized[5] as u16) << 8) | (serialized[4] as u16);
        assert_eq!(length, 3);
    }

    #[test]
    fn test_checksum_calculation() {
        // Test known checksum values
        let data = [0xFD, 0xFC, 0xFB, 0xFA, 0x03, 0x00, 0x01, 0x10, 0x20];
        let checksum = calculate_checksum(&data);
        assert!(checksum > 0); // Should produce a non-zero checksum
    }

    #[test]
    fn test_checksum_validation_success() {
        // Property 2: Checksum Validation Success
        // For any frame with a valid CRC16 checksum, the checksum verification
        // should succeed and the frame should be accepted for processing.
        let data = [0x10, 0x20, 0x30];
        let frame = Frame::new(0x01, &data).unwrap();
        let serialized = frame.serialize();

        // Extract checksum from serialized frame
        let frame_len = frame.serialized_len();
        let checksum = ((serialized[frame_len - 1] as u16) << 8)
            | (serialized[frame_len - 2] as u16);

        // Validate checksum
        assert!(validate_checksum(&serialized[..frame_len - 2], checksum));
    }

    #[test]
    fn test_checksum_validation_failure() {
        // Property 3: Checksum Validation Failure
        // For any frame where a single byte is corrupted, the checksum verification
        // should fail and the frame should be rejected.
        let data = [0x10, 0x20, 0x30];
        let frame = Frame::new(0x01, &data).unwrap();
        let mut serialized = frame.serialize();

        // Extract original checksum
        let frame_len = frame.serialized_len();
        let original_checksum = ((serialized[frame_len - 1] as u16) << 8)
            | (serialized[frame_len - 2] as u16);

        // Corrupt a byte in the data
        serialized[7] ^= 0xFF;

        // Recalculate checksum - should not match
        let new_checksum = calculate_checksum(&serialized[..frame_len - 2]);
        assert_ne!(new_checksum, original_checksum);
    }

    #[test]
    fn test_frame_round_trip_consistency() {
        // Property 1: Frame Round-Trip Consistency
        // For any valid frame with presence data, parsing it and then serializing it
        // should produce an equivalent frame with identical bytes.
        let original_data = [0x01, 0x64, 0x00, 0x96, 0x00, 0x32, 0x1E];
        let frame = Frame::new(0x01, &original_data).unwrap();
        let serialized = frame.serialize();

        // Parse the serialized frame
        let parsed = parse_frame(&serialized[..frame.serialized_len()]).unwrap();

        // Verify parsed frame matches original
        assert_eq!(parsed.command, frame.command);
        assert_eq!(parsed.data_len, frame.data_len);
        assert_eq!(&parsed.data[..parsed.data_len], &frame.data[..frame.data_len]);

        // Re-serialize and verify bytes match
        let reserialized = parsed.serialize();
        assert_eq!(&serialized[..frame.serialized_len()], &reserialized[..frame.serialized_len()]);
    }

    #[test]
    fn test_frame_parsing_valid_frame() {
        let data = [0x10, 0x20, 0x30];
        let frame = Frame::new(0x02, &data).unwrap();
        let serialized = frame.serialize();
        let frame_len = frame.serialized_len();

        let parsed = parse_frame(&serialized[..frame_len]).unwrap();
        assert_eq!(parsed.command, 0x02);
        assert_eq!(parsed.data_len, 3);
        assert_eq!(&parsed.data[..3], &data);
    }

    #[test]
    fn test_frame_parsing_invalid_header() {
        let mut buffer = [0u8; 20];
        buffer[0] = 0xFF; // Invalid header
        buffer[1] = 0xFF;
        buffer[2] = 0xFF;
        buffer[3] = 0xFF;

        let result = parse_frame(&buffer);
        assert_eq!(result, Err(Error::InvalidFrame));
    }

    #[test]
    fn test_frame_parsing_too_short() {
        let buffer = [0u8; 8]; // Too short for minimum frame
        let result = parse_frame(&buffer);
        assert_eq!(result, Err(Error::InvalidFrame));
    }

    #[test]
    fn test_frame_parsing_invalid_checksum() {
        // Property 3: Checksum Validation Failure
        // Create a frame and corrupt it
        let data = [0x10, 0x20];
        let frame = Frame::new(0x01, &data).unwrap();
        let mut serialized = frame.serialize();
        let frame_len = frame.serialized_len();

        // Corrupt the checksum
        serialized[frame_len - 1] ^= 0xFF;

        let result = parse_frame(&serialized[..frame_len]);
        assert_eq!(result, Err(Error::InvalidChecksum));
    }

    #[test]
    fn test_frame_new_with_oversized_data() {
        let oversized_data = [0u8; frame::MAX_DATA_SIZE + 1];
        let result = Frame::new(0x01, &oversized_data);
        assert_eq!(result, Err(Error::InvalidFrame));
    }

    #[test]
    fn test_frame_serialized_len() {
        let frame_empty = Frame::new(0x01, &[]).unwrap();
        assert_eq!(frame_empty.serialized_len(), 4 + 2 + 1 + 0 + 2); // 9

        let frame_with_data = Frame::new(0x01, &[0x10, 0x20, 0x30]).unwrap();
        assert_eq!(frame_with_data.serialized_len(), 4 + 2 + 1 + 3 + 2); // 12
    }

    #[test]
    fn test_malformed_frame_handling() {
        // Property 10: Malformed Frame Handling
        // For any frame with missing required fields or invalid structure,
        // the driver should return an error instead of panicking.

        // Frame too short
        let short_buffer = [0xFD, 0xFC, 0xFB, 0xFA];
        assert_eq!(parse_frame(&short_buffer), Err(Error::InvalidFrame));

        // Invalid header
        let invalid_header = [0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x01];
        assert_eq!(parse_frame(&invalid_header), Err(Error::InvalidFrame));

        // Corrupted checksum
        let data = [0x10, 0x20];
        let frame = Frame::new(0x01, &data).unwrap();
        let mut serialized = frame.serialize();
        let frame_len = frame.serialized_len();
        serialized[frame_len - 1] = 0xFF;
        serialized[frame_len - 2] = 0xFF;

        assert_eq!(parse_frame(&serialized[..frame_len]), Err(Error::InvalidChecksum));
    }

    // UART communication layer tests
    use crate::uart_traits::{DelayMs, Read, Write};

    /// Mock UART for testing
    struct MockUart {
        tx_buffer: [u8; 256],
        tx_pos: usize,
        rx_buffer: [u8; 256],
        rx_pos: usize,
        rx_read_pos: usize,
    }

    impl MockUart {
        fn new() -> Self {
            Self {
                tx_buffer: [0u8; 256],
                tx_pos: 0,
                rx_buffer: [0u8; 256],
                rx_pos: 0,
                rx_read_pos: 0,
            }
        }

        fn set_rx_data(&mut self, data: &[u8]) {
            self.rx_buffer[..data.len()].copy_from_slice(data);
            self.rx_pos = data.len();
            self.rx_read_pos = 0;
        }

        fn get_tx_data(&self) -> &[u8] {
            &self.tx_buffer[..self.tx_pos]
        }
    }

    impl Write for MockUart {
        type Error = ();

        fn write(&mut self, byte: u8) -> Result<(), Self::Error> {
            if self.tx_pos >= self.tx_buffer.len() {
                return Err(());
            }
            self.tx_buffer[self.tx_pos] = byte;
            self.tx_pos += 1;
            Ok(())
        }
    }

    impl Read for MockUart {
        type Error = ();

        fn read(&mut self) -> Result<Option<u8>, Self::Error> {
            if self.rx_read_pos < self.rx_pos {
                let byte = self.rx_buffer[self.rx_read_pos];
                self.rx_read_pos += 1;
                Ok(Some(byte))
            } else {
                Ok(None)
            }
        }
    }

    /// Mock delay for testing
    struct MockDelay;

    impl DelayMs for MockDelay {
        fn delay_ms(&mut self, _ms: u32) {
            // No-op for testing
        }
    }

    #[test]
    fn test_ld2410_creation() {
        let uart = MockUart::new();
        let delay = MockDelay;

        let driver = LD2410::new(uart, delay);
        assert_eq!(driver.timeout_ms, 1000);
    }

    #[test]
    fn test_ld2410_with_custom_timeout() {
        let uart = MockUart::new();
        let delay = MockDelay;

        let driver = LD2410::with_timeout(uart, delay, 5000);
        assert_eq!(driver.timeout_ms, 5000);
    }

    #[test]
    fn test_send_frame() {
        let uart = MockUart::new();
        let delay = MockDelay;
        let mut driver = LD2410::new(uart, delay);

        let frame = Frame::new(0x01, &[0x10, 0x20, 0x30]).unwrap();
        let result = driver.send_frame(&frame);

        assert!(result.is_ok());

        // Verify transmitted data
        let tx_data = driver.uart.get_tx_data();
        assert!(tx_data.len() > 0);
        assert_eq!(&tx_data[0..4], &frame::HEADER);
    }

    #[test]
    fn test_send_frame_empty_data() {
        let uart = MockUart::new();
        let delay = MockDelay;
        let mut driver = LD2410::new(uart, delay);

        let frame = Frame::new(0x02, &[]).unwrap();
        let result = driver.send_frame(&frame);

        assert!(result.is_ok());

        let tx_data = driver.uart.get_tx_data();
        assert_eq!(&tx_data[0..4], &frame::HEADER);
        assert_eq!(tx_data[6], 0x02); // Command
    }

    #[test]
    fn test_receive_frame_valid() {
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Create a valid frame and set it as RX data
        let frame = Frame::new(0x01, &[0x10, 0x20]).unwrap();
        let serialized = frame.serialize();
        let frame_len = frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.receive_frame();

        assert!(result.is_ok());
        let received = result.unwrap();
        assert_eq!(received.command, 0x01);
        assert_eq!(received.data_len, 2);
        assert_eq!(&received.data[..2], &[0x10, 0x20]);
    }

    #[test]
    fn test_receive_frame_timeout() {
        let uart = MockUart::new();
        let delay = MockDelay;

        let mut driver = LD2410::with_timeout(uart, delay, 10);
        let result = driver.receive_frame();

        // Should timeout since no data is available
        assert_eq!(result, Err(Error::Timeout));
    }

    #[test]
    fn test_receive_frame_invalid_checksum() {
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Create a frame and corrupt the checksum
        let frame = Frame::new(0x01, &[0x10, 0x20]).unwrap();
        let mut serialized = frame.serialize();
        let frame_len = frame.serialized_len();
        serialized[frame_len - 1] ^= 0xFF;

        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.receive_frame();

        assert_eq!(result, Err(Error::InvalidChecksum));
    }

    #[test]
    fn test_receive_frame_invalid_header() {
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Create invalid frame with wrong header
        let mut buffer = [0u8; 20];
        buffer[0] = 0xFF;
        buffer[1] = 0xFF;
        buffer[2] = 0xFF;
        buffer[3] = 0xFF;
        buffer[4] = 0x01;
        buffer[5] = 0x00;
        buffer[6] = 0x01;

        uart.set_rx_data(&buffer[..10]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.receive_frame();

        assert_eq!(result, Err(Error::InvalidFrame));
    }

    #[test]
    fn test_frame_round_trip_via_uart() {
        // Test sending and receiving a frame through the driver
        let uart = MockUart::new();
        let delay = MockDelay;
        let mut driver = LD2410::new(uart, delay);

        // Send a frame
        let original_frame = Frame::new(0x03, &[0x64, 0x00, 0x32, 0x1E]).unwrap();
        let send_result = driver.send_frame(&original_frame);
        assert!(send_result.is_ok());

        // Get the transmitted data
        let tx_data = driver.uart.get_tx_data().to_vec();

        // Create a new driver with the transmitted data as RX
        let mut uart2 = MockUart::new();
        uart2.set_rx_data(&tx_data);
        let delay2 = MockDelay;
        let mut driver2 = LD2410::new(uart2, delay2);

        // Receive the frame
        let receive_result = driver2.receive_frame();
        assert!(receive_result.is_ok());

        let received_frame = receive_result.unwrap();
        assert_eq!(received_frame.command, original_frame.command);
        assert_eq!(received_frame.data_len, original_frame.data_len);
        assert_eq!(
            &received_frame.data[..received_frame.data_len],
            &original_frame.data[..original_frame.data_len]
        );
    }

    // Presence data reading tests
    #[test]
    fn test_read_presence_with_motion_and_static() {
        // Property 8: Presence Data Field Extraction
        // For any valid sensor response frame, parsing it should correctly extract
        // presence state, motion distance, static distance, motion energy, and static energy.
        
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Create a response frame with presence data
        // Presence: 0x01 (detected)
        // Motion distance: 0x64, 0x00 (100 cm, little-endian)
        // Static distance: 0x96, 0x00 (150 cm, little-endian)
        // Motion energy: 0x32 (50)
        // Static energy: 0x1E (30)
        let response_data = [0x01, 0x64, 0x00, 0x96, 0x00, 0x32, 0x1E];
        let response_frame = Frame::new(0x01, &response_data).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();

        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.read_presence();

        assert!(result.is_ok());
        let presence_data = result.unwrap();

        assert_eq!(presence_data.presence, true);
        assert_eq!(presence_data.motion_distance_cm, 100);
        assert_eq!(presence_data.static_distance_cm, 150);
        assert_eq!(presence_data.motion_energy, 50);
        assert_eq!(presence_data.static_energy, 30);
    }

    #[test]
    fn test_read_presence_no_detection() {
        // Property 4: Distance Invariant
        // When no motion is detected, motion_distance_cm must be 0 and motion_energy must be 0.
        // When no static presence is detected, static_distance_cm must be 0 and static_energy must be 0.
        
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Create a response frame with no presence
        // Presence: 0x00 (not detected)
        // Motion distance: 0x00, 0x00 (0 cm)
        // Static distance: 0x00, 0x00 (0 cm)
        // Motion energy: 0x00 (0)
        // Static energy: 0x00 (0)
        let response_data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        let response_frame = Frame::new(0x01, &response_data).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();

        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.read_presence();

        assert!(result.is_ok());
        let presence_data = result.unwrap();

        assert_eq!(presence_data.presence, false);
        assert_eq!(presence_data.motion_distance_cm, 0);
        assert_eq!(presence_data.static_distance_cm, 0);
        assert_eq!(presence_data.motion_energy, 0);
        assert_eq!(presence_data.static_energy, 0);
    }

    #[test]
    fn test_read_presence_motion_only() {
        // Test when only motion is detected (static presence is 0)
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Presence: 0x01 (detected)
        // Motion distance: 0xC8, 0x00 (200 cm)
        // Static distance: 0x00, 0x00 (0 cm)
        // Motion energy: 0x64 (100)
        // Static energy: 0x00 (0)
        let response_data = [0x01, 0xC8, 0x00, 0x00, 0x00, 0x64, 0x00];
        let response_frame = Frame::new(0x01, &response_data).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();

        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.read_presence();

        assert!(result.is_ok());
        let presence_data = result.unwrap();

        assert_eq!(presence_data.presence, true);
        assert_eq!(presence_data.motion_distance_cm, 200);
        assert_eq!(presence_data.static_distance_cm, 0);
        assert_eq!(presence_data.motion_energy, 100);
        assert_eq!(presence_data.static_energy, 0);
    }

    #[test]
    fn test_read_presence_static_only() {
        // Test when only static presence is detected (motion is 0)
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Presence: 0x01 (detected)
        // Motion distance: 0x00, 0x00 (0 cm)
        // Static distance: 0x2C, 0x01 (300 cm)
        // Motion energy: 0x00 (0)
        // Static energy: 0x50 (80)
        let response_data = [0x01, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x50];
        let response_frame = Frame::new(0x01, &response_data).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();

        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.read_presence();

        assert!(result.is_ok());
        let presence_data = result.unwrap();

        assert_eq!(presence_data.presence, true);
        assert_eq!(presence_data.motion_distance_cm, 0);
        assert_eq!(presence_data.static_distance_cm, 300);
        assert_eq!(presence_data.motion_energy, 0);
        assert_eq!(presence_data.static_energy, 80);
    }

    #[test]
    fn test_read_presence_energy_bounds() {
        // Property 5: Energy Level Bounds
        // For any presence data read from the sensor, motion energy and static energy
        // values must be within the range [0, 255].
        
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Test with maximum energy values (255)
        let response_data = [0x01, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF];
        let response_frame = Frame::new(0x01, &response_data).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();

        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.read_presence();

        assert!(result.is_ok());
        let presence_data = result.unwrap();

        // u8 values are always in range [0, 255]
        assert_eq!(presence_data.motion_energy, 255);
        assert_eq!(presence_data.static_energy, 255);
    }

    #[test]
    fn test_read_presence_invalid_frame_too_short() {
        // Property 10: Malformed Frame Handling
        // For any frame with missing required fields, the driver should return an error.
        
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Create a response frame with insufficient data (only 3 bytes instead of 7)
        let response_data = [0x01, 0x64, 0x00];
        let response_frame = Frame::new(0x01, &response_data).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();

        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.read_presence();

        assert_eq!(result, Err(Error::InvalidFrame));
    }

    #[test]
    fn test_read_presence_timeout() {
        // Test timeout when no response is received
        let uart = MockUart::new();
        let delay = MockDelay;

        let mut driver = LD2410::with_timeout(uart, delay, 10);
        let result = driver.read_presence();

        assert_eq!(result, Err(Error::Timeout));
    }

    #[test]
    fn test_read_presence_invalid_checksum() {
        // Test that invalid checksum in response is rejected
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_data = [0x01, 0x64, 0x00, 0x96, 0x00, 0x32, 0x1E];
        let response_frame = Frame::new(0x01, &response_data).unwrap();
        let mut serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();

        // Corrupt the checksum
        serialized[frame_len - 1] ^= 0xFF;

        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.read_presence();

        assert_eq!(result, Err(Error::InvalidChecksum));
    }

    // Configuration method tests
    #[test]
    fn test_set_max_distance() {
        // Test setting maximum detection distance
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Create a mock response frame for the set_max_distance command
        let response_frame = Frame::new(0x03, &[0x00]).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_max_distance(600);

        assert!(result.is_ok());

        // Verify the command was sent correctly
        let tx_data = driver.uart.get_tx_data();
        assert!(tx_data.len() > 0);
        assert_eq!(&tx_data[0..4], &frame::HEADER);
        assert_eq!(tx_data[6], 0x03); // Command
        // Distance 600 = 0x58, 0x02 (little-endian)
        assert_eq!(tx_data[7], 0x58);
        assert_eq!(tx_data[8], 0x02);
    }

    #[test]
    fn test_set_max_distance_100cm() {
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x03, &[0x00]).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_max_distance(100);

        assert!(result.is_ok());

        let tx_data = driver.uart.get_tx_data();
        assert_eq!(tx_data[7], 0x64);
        assert_eq!(tx_data[8], 0x00);
    }

    #[test]
    fn test_set_max_distance_300cm() {
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x03, &[0x00]).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_max_distance(300);

        assert!(result.is_ok());

        let tx_data = driver.uart.get_tx_data();
        assert_eq!(tx_data[7], 0x2C);
        assert_eq!(tx_data[8], 0x01);
    }

    #[test]
    fn test_set_max_distance_1000cm() {
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x03, &[0x00]).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_max_distance(1000);

        assert!(result.is_ok());

        let tx_data = driver.uart.get_tx_data();
        assert_eq!(tx_data[7], 0xE8);
        assert_eq!(tx_data[8], 0x03);
    }

    #[test]
    fn test_set_motion_sensitivity_valid() {
        // Test setting motion sensitivity with valid values
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x04, &[0x00]).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_motion_sensitivity(50);

        assert!(result.is_ok());

        let tx_data = driver.uart.get_tx_data();
        assert_eq!(&tx_data[0..4], &frame::HEADER);
        assert_eq!(tx_data[6], 0x04); // Command
        assert_eq!(tx_data[7], 50); // Sensitivity value
    }

    #[test]
    fn test_set_motion_sensitivity_0() {
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x04, &[0x00]).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_motion_sensitivity(0);

        assert!(result.is_ok());

        let tx_data = driver.uart.get_tx_data();
        assert_eq!(tx_data[7], 0);
    }

    #[test]
    fn test_set_motion_sensitivity_50() {
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x04, &[0x00]).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_motion_sensitivity(50);

        assert!(result.is_ok());

        let tx_data = driver.uart.get_tx_data();
        assert_eq!(tx_data[7], 50);
    }

    #[test]
    fn test_set_motion_sensitivity_100() {
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x04, &[0x00]).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_motion_sensitivity(100);

        assert!(result.is_ok());

        let tx_data = driver.uart.get_tx_data();
        assert_eq!(tx_data[7], 100);
    }

    #[test]
    fn test_set_motion_sensitivity_invalid() {
        // Property 6: Configuration Parameter Validation
        // For any configuration parameter outside the valid range,
        // the driver should reject it and return an error.
        
        let uart = MockUart::new();
        let delay = MockDelay;

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_motion_sensitivity(101); // Out of range

        assert_eq!(result, Err(Error::InvalidFrame));
    }

    #[test]
    fn test_set_static_sensitivity_valid() {
        // Test setting static sensitivity with valid values
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x05, &[0x00]).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_static_sensitivity(40);

        assert!(result.is_ok());

        let tx_data = driver.uart.get_tx_data();
        assert_eq!(&tx_data[0..4], &frame::HEADER);
        assert_eq!(tx_data[6], 0x05); // Command
        assert_eq!(tx_data[7], 40); // Sensitivity value
    }

    #[test]
    fn test_set_static_sensitivity_0() {
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x05, &[0x00]).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_static_sensitivity(0);

        assert!(result.is_ok());

        let tx_data = driver.uart.get_tx_data();
        assert_eq!(tx_data[7], 0);
    }

    #[test]
    fn test_set_static_sensitivity_50() {
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x05, &[0x00]).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_static_sensitivity(50);

        assert!(result.is_ok());

        let tx_data = driver.uart.get_tx_data();
        assert_eq!(tx_data[7], 50);
    }

    #[test]
    fn test_set_static_sensitivity_100() {
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x05, &[0x00]).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_static_sensitivity(100);

        assert!(result.is_ok());

        let tx_data = driver.uart.get_tx_data();
        assert_eq!(tx_data[7], 100);
    }

    #[test]
    fn test_set_static_sensitivity_invalid() {
        // Property 6: Configuration Parameter Validation
        // For any configuration parameter outside the valid range,
        // the driver should reject it and return an error.
        
        let uart = MockUart::new();
        let delay = MockDelay;

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_static_sensitivity(101); // Out of range

        assert_eq!(result, Err(Error::InvalidFrame));
    }

    #[test]
    fn test_save_config() {
        // Test saving configuration to non-volatile memory
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x06, &[0x00]).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.save_config();

        assert!(result.is_ok());

        let tx_data = driver.uart.get_tx_data();
        assert_eq!(&tx_data[0..4], &frame::HEADER);
        assert_eq!(tx_data[6], 0x06); // Command
        // No data payload for save_config
    }

    #[test]
    fn test_set_max_distance_timeout() {
        // Test timeout when no response is received
        let uart = MockUart::new();
        let delay = MockDelay;

        let mut driver = LD2410::with_timeout(uart, delay, 10);
        let result = driver.set_max_distance(600);

        assert_eq!(result, Err(Error::Timeout));
    }

    #[test]
    fn test_set_motion_sensitivity_timeout() {
        // Test timeout when no response is received
        let uart = MockUart::new();
        let delay = MockDelay;

        let mut driver = LD2410::with_timeout(uart, delay, 10);
        let result = driver.set_motion_sensitivity(50);

        assert_eq!(result, Err(Error::Timeout));
    }

    #[test]
    fn test_set_static_sensitivity_timeout() {
        // Test timeout when no response is received
        let uart = MockUart::new();
        let delay = MockDelay;

        let mut driver = LD2410::with_timeout(uart, delay, 10);
        let result = driver.set_static_sensitivity(40);

        assert_eq!(result, Err(Error::Timeout));
    }

    #[test]
    fn test_save_config_timeout() {
        // Test timeout when no response is received
        let uart = MockUart::new();
        let delay = MockDelay;

        let mut driver = LD2410::with_timeout(uart, delay, 10);
        let result = driver.save_config();

        assert_eq!(result, Err(Error::Timeout));
    }

    #[test]
    fn test_set_max_distance_invalid_checksum() {
        // Test that invalid checksum in response is rejected
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x03, &[0x00]).unwrap();
        let mut serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();

        // Corrupt the checksum
        serialized[frame_len - 1] ^= 0xFF;

        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.set_max_distance(600);

        assert_eq!(result, Err(Error::InvalidChecksum));
    }

    #[test]
    fn test_configuration_sequence() {
        // Test a sequence of configuration operations
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Set up responses for multiple commands
        let mut responses = [0u8; 256];
        let mut pos = 0;
        
        // Response for set_max_distance
        let resp1 = Frame::new(0x03, &[0x00]).unwrap();
        let ser1 = resp1.serialize();
        let len1 = resp1.serialized_len();
        responses[pos..pos + len1].copy_from_slice(&ser1[..len1]);
        pos += len1;

        // Response for set_motion_sensitivity
        let resp2 = Frame::new(0x04, &[0x00]).unwrap();
        let ser2 = resp2.serialize();
        let len2 = resp2.serialized_len();
        responses[pos..pos + len2].copy_from_slice(&ser2[..len2]);
        pos += len2;

        // Response for set_static_sensitivity
        let resp3 = Frame::new(0x05, &[0x00]).unwrap();
        let ser3 = resp3.serialize();
        let len3 = resp3.serialized_len();
        responses[pos..pos + len3].copy_from_slice(&ser3[..len3]);
        pos += len3;

        // Response for save_config
        let resp4 = Frame::new(0x06, &[0x00]).unwrap();
        let ser4 = resp4.serialize();
        let len4 = resp4.serialized_len();
        responses[pos..pos + len4].copy_from_slice(&ser4[..len4]);
        pos += len4;

        uart.set_rx_data(&responses[..pos]);

        let mut driver = LD2410::new(uart, delay);

        // Execute configuration sequence
        assert!(driver.set_max_distance(600).is_ok());
        assert!(driver.set_motion_sensitivity(50).is_ok());
        assert!(driver.set_static_sensitivity(40).is_ok());
        assert!(driver.save_config().is_ok());
    }

    // Utility method tests
    #[test]
    fn test_reset() {
        // Test reset command
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x07, &[0x00]).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.reset();

        assert!(result.is_ok());

        let tx_data = driver.uart.get_tx_data();
        assert_eq!(&tx_data[0..4], &frame::HEADER);
        assert_eq!(tx_data[6], 0x07); // Command
    }

    #[test]
    fn test_reset_timeout() {
        // Test timeout when no response is received
        let uart = MockUart::new();
        let delay = MockDelay;

        let mut driver = LD2410::with_timeout(uart, delay, 10);
        let result = driver.reset();

        assert_eq!(result, Err(Error::Timeout));
    }

    #[test]
    fn test_reset_invalid_checksum() {
        // Test that invalid checksum in response is rejected
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_frame = Frame::new(0x07, &[0x00]).unwrap();
        let mut serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();

        // Corrupt the checksum
        serialized[frame_len - 1] ^= 0xFF;

        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.reset();

        assert_eq!(result, Err(Error::InvalidChecksum));
    }

    #[test]
    fn test_get_version() {
        // Test getting firmware version
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Create a response frame with version data
        // Version: 1.2.3
        let response_data = [0x01, 0x02, 0x03];
        let response_frame = Frame::new(0x08, &response_data).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.get_version();

        assert!(result.is_ok());
        let (major, minor, patch) = result.unwrap();
        assert_eq!(major, 1);
        assert_eq!(minor, 2);
        assert_eq!(patch, 3);
    }

    #[test]
    fn test_get_version_different_values() {
        // Test with different version values
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_data = [0x02, 0x05, 0x0A];
        let response_frame = Frame::new(0x08, &response_data).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.get_version();

        assert!(result.is_ok());
        let (major, minor, patch) = result.unwrap();
        assert_eq!(major, 2);
        assert_eq!(minor, 5);
        assert_eq!(patch, 10);
    }

    #[test]
    fn test_get_version_timeout() {
        // Test timeout when no response is received
        let uart = MockUart::new();
        let delay = MockDelay;

        let mut driver = LD2410::with_timeout(uart, delay, 10);
        let result = driver.get_version();

        assert_eq!(result, Err(Error::Timeout));
    }

    #[test]
    fn test_get_version_invalid_checksum() {
        // Test that invalid checksum in response is rejected
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_data = [0x01, 0x02, 0x03];
        let response_frame = Frame::new(0x08, &response_data).unwrap();
        let mut serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();

        // Corrupt the checksum
        serialized[frame_len - 1] ^= 0xFF;

        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.get_version();

        assert_eq!(result, Err(Error::InvalidChecksum));
    }

    #[test]
    fn test_get_version_invalid_frame_too_short() {
        // Test that frames with insufficient data are rejected
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_data = [0x01, 0x02]; // Only 2 bytes instead of 3
        let response_frame = Frame::new(0x08, &response_data).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.get_version();

        assert_eq!(result, Err(Error::InvalidFrame));
    }

    #[test]
    fn test_read_config() {
        // Test reading sensor configuration
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Create a response frame with config data
        // Max distance: 600 cm (0x58, 0x02 in little-endian)
        // Motion sensitivity: 50
        // Static sensitivity: 40
        let response_data = [0x58, 0x02, 0x32, 0x28];
        let response_frame = Frame::new(0x02, &response_data).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.read_config();

        assert!(result.is_ok());
        let config = result.unwrap();
        assert_eq!(config.max_distance_cm, 600);
        assert_eq!(config.motion_sensitivity, 50);
        assert_eq!(config.static_sensitivity, 40);
    }

    #[test]
    fn test_read_config_different_values() {
        // Test with different configuration values
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Max distance: 300 cm (0x2C, 0x01 in little-endian)
        // Motion sensitivity: 75
        // Static sensitivity: 60
        let response_data = [0x2C, 0x01, 0x4B, 0x3C];
        let response_frame = Frame::new(0x02, &response_data).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.read_config();

        assert!(result.is_ok());
        let config = result.unwrap();
        assert_eq!(config.max_distance_cm, 300);
        assert_eq!(config.motion_sensitivity, 75);
        assert_eq!(config.static_sensitivity, 60);
    }

    #[test]
    fn test_read_config_timeout() {
        // Test timeout when no response is received
        let uart = MockUart::new();
        let delay = MockDelay;

        let mut driver = LD2410::with_timeout(uart, delay, 10);
        let result = driver.read_config();

        assert_eq!(result, Err(Error::Timeout));
    }

    #[test]
    fn test_read_config_invalid_checksum() {
        // Test that invalid checksum in response is rejected
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_data = [0x58, 0x02, 0x32, 0x28];
        let response_frame = Frame::new(0x02, &response_data).unwrap();
        let mut serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();

        // Corrupt the checksum
        serialized[frame_len - 1] ^= 0xFF;

        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.read_config();

        assert_eq!(result, Err(Error::InvalidChecksum));
    }

    #[test]
    fn test_read_config_invalid_frame_too_short() {
        // Test that frames with insufficient data are rejected
        let mut uart = MockUart::new();
        let delay = MockDelay;

        let response_data = [0x58, 0x02, 0x32]; // Only 3 bytes instead of 4
        let response_frame = Frame::new(0x02, &response_data).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.read_config();

        assert_eq!(result, Err(Error::InvalidFrame));
    }

    #[test]
    fn test_read_config_100cm() {
        // Test with 100 cm max distance
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Max distance: 100 cm (0x64, 0x00 in little-endian)
        let response_data = [0x64, 0x00, 0x32, 0x28];
        let response_frame = Frame::new(0x02, &response_data).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.read_config();

        assert!(result.is_ok());
        let config = result.unwrap();
        assert_eq!(config.max_distance_cm, 100);
    }

    #[test]
    fn test_read_config_1000cm() {
        // Test with 1000 cm max distance
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Max distance: 1000 cm (0xE8, 0x03 in little-endian)
        let response_data = [0xE8, 0x03, 0x32, 0x28];
        let response_frame = Frame::new(0x02, &response_data).unwrap();
        let serialized = response_frame.serialize();
        let frame_len = response_frame.serialized_len();
        uart.set_rx_data(&serialized[..frame_len]);

        let mut driver = LD2410::new(uart, delay);
        let result = driver.read_config();

        assert!(result.is_ok());
        let config = result.unwrap();
        assert_eq!(config.max_distance_cm, 1000);
    }

    #[test]
    fn test_utility_methods_sequence() {
        // Test a sequence of utility method calls
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Set up responses for multiple commands
        let mut responses = [0u8; 256];
        let mut pos = 0;
        
        // Response for get_version
        let resp1 = Frame::new(0x08, &[0x01, 0x02, 0x03]).unwrap();
        let ser1 = resp1.serialize();
        let len1 = resp1.serialized_len();
        responses[pos..pos + len1].copy_from_slice(&ser1[..len1]);
        pos += len1;

        // Response for read_config
        let resp2 = Frame::new(0x02, &[0x58, 0x02, 0x32, 0x28]).unwrap();
        let ser2 = resp2.serialize();
        let len2 = resp2.serialized_len();
        responses[pos..pos + len2].copy_from_slice(&ser2[..len2]);
        pos += len2;

        // Response for reset
        let resp3 = Frame::new(0x07, &[0x00]).unwrap();
        let ser3 = resp3.serialize();
        let len3 = resp3.serialized_len();
        responses[pos..pos + len3].copy_from_slice(&ser3[..len3]);
        pos += len3;

        uart.set_rx_data(&responses[..pos]);

        let mut driver = LD2410::new(uart, delay);

        // Execute utility method sequence
        let version_result = driver.get_version();
        assert!(version_result.is_ok());
        let (major, minor, patch) = version_result.unwrap();
        assert_eq!(major, 1);
        assert_eq!(minor, 2);
        assert_eq!(patch, 3);

        let config_result = driver.read_config();
        assert!(config_result.is_ok());
        let config = config_result.unwrap();
        assert_eq!(config.max_distance_cm, 600);
        assert_eq!(config.motion_sensitivity, 50);
        assert_eq!(config.static_sensitivity, 40);

        let reset_result = driver.reset();
        assert!(reset_result.is_ok());
    }

    #[test]
    fn test_error_recovery_after_invalid_checksum() {
        // Property 9: Error Recovery State
        // For any error condition (invalid checksum, timeout, malformed frame),
        // after the error is returned, the driver should be able to process
        // a subsequent valid frame without requiring manual intervention.
        
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // First, create an invalid frame with corrupted checksum
        let invalid_frame = Frame::new(0x01, &[0x10, 0x20]).unwrap();
        let mut invalid_serialized = invalid_frame.serialize();
        let invalid_len = invalid_frame.serialized_len();
        invalid_serialized[invalid_len - 1] ^= 0xFF; // Corrupt checksum

        // Then, create a valid frame
        let valid_frame = Frame::new(0x01, &[0x01, 0x64, 0x00, 0x96, 0x00, 0x32, 0x1E]).unwrap();
        let valid_serialized = valid_frame.serialize();
        let valid_len = valid_frame.serialized_len();

        // Combine both frames in the RX buffer
        let mut combined = [0u8; 256];
        combined[..invalid_len].copy_from_slice(&invalid_serialized[..invalid_len]);
        combined[invalid_len..invalid_len + valid_len].copy_from_slice(&valid_serialized[..valid_len]);

        uart.set_rx_data(&combined[..invalid_len + valid_len]);

        let mut driver = LD2410::new(uart, delay);

        // First receive should fail with invalid checksum
        let first_result = driver.receive_frame();
        assert_eq!(first_result, Err(Error::InvalidChecksum));

        // Second receive should succeed with the valid frame
        let second_result = driver.receive_frame();
        assert!(second_result.is_ok());
        let received = second_result.unwrap();
        assert_eq!(received.command, 0x01);
        assert_eq!(received.data_len, 7);
    }

    #[test]
    fn test_error_recovery_after_timeout() {
        // Test that driver can recover after a timeout error
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // First, set up no data (will timeout)
        uart.set_rx_data(&[]);

        let mut driver = LD2410::with_timeout(uart, delay, 10);

        // First receive should timeout
        let first_result = driver.receive_frame();
        assert_eq!(first_result, Err(Error::Timeout));

        // Now set up a valid frame for the next attempt
        let valid_frame = Frame::new(0x01, &[0x01, 0x64, 0x00, 0x96, 0x00, 0x32, 0x1E]).unwrap();
        let valid_serialized = valid_frame.serialize();
        let valid_len = valid_frame.serialized_len();

        driver.uart.set_rx_data(&valid_serialized[..valid_len]);

        // Second receive should succeed
        let second_result = driver.receive_frame();
        assert!(second_result.is_ok());
        let received = second_result.unwrap();
        assert_eq!(received.command, 0x01);
    }

    #[test]
    fn test_error_recovery_after_malformed_frame() {
        // Test that driver can recover after receiving a malformed frame
        // Note: When an invalid header is detected, the driver returns an error.
        // The driver state remains valid for the next operation.
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // First, set up a malformed frame (invalid header)
        let mut malformed = [0u8; 20];
        malformed[0] = 0xFF;
        malformed[1] = 0xFF;
        malformed[2] = 0xFF;
        malformed[3] = 0xFF;
        malformed[4] = 0x01;
        malformed[5] = 0x00;
        malformed[6] = 0x01;

        uart.set_rx_data(&malformed[..10]);

        let mut driver = LD2410::new(uart, delay);

        // First receive should fail with invalid frame
        let first_result = driver.receive_frame();
        assert_eq!(first_result, Err(Error::InvalidFrame));

        // Now set up a valid frame for the next attempt
        let valid_frame = Frame::new(0x01, &[0x01, 0x64, 0x00, 0x96, 0x00, 0x32, 0x1E]).unwrap();
        let valid_serialized = valid_frame.serialize();
        let valid_len = valid_frame.serialized_len();

        driver.uart.set_rx_data(&valid_serialized[..valid_len]);

        // Second receive should succeed with the valid frame
        let second_result = driver.receive_frame();
        assert!(second_result.is_ok());
        let received = second_result.unwrap();
        assert_eq!(received.command, 0x01);
    }

    #[test]
    fn test_error_recovery_read_presence_after_error() {
        // Test that read_presence can recover after an error
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // First, create an invalid frame
        let invalid_frame = Frame::new(0x01, &[0x10]).unwrap();
        let mut invalid_serialized = invalid_frame.serialize();
        let invalid_len = invalid_frame.serialized_len();
        invalid_serialized[invalid_len - 1] ^= 0xFF; // Corrupt checksum

        // Then, create a valid presence response
        let valid_response = Frame::new(0x01, &[0x01, 0x64, 0x00, 0x96, 0x00, 0x32, 0x1E]).unwrap();
        let valid_serialized = valid_response.serialize();
        let valid_len = valid_response.serialized_len();

        // Combine both frames
        let mut combined = [0u8; 256];
        combined[..invalid_len].copy_from_slice(&invalid_serialized[..invalid_len]);
        combined[invalid_len..invalid_len + valid_len].copy_from_slice(&valid_serialized[..valid_len]);

        uart.set_rx_data(&combined[..invalid_len + valid_len]);

        let mut driver = LD2410::new(uart, delay);

        // First receive_frame should fail
        let first_result = driver.receive_frame();
        assert_eq!(first_result, Err(Error::InvalidChecksum));

        // Now read_presence should work with the valid frame
        let presence_result = driver.read_presence();
        assert!(presence_result.is_ok());
        let presence_data = presence_result.unwrap();
        assert_eq!(presence_data.presence, true);
        assert_eq!(presence_data.motion_distance_cm, 100);
    }

    #[test]
    fn test_error_recovery_configuration_after_error() {
        // Test that configuration methods can recover after an error
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // First, create an invalid frame
        let invalid_frame = Frame::new(0x03, &[0x10]).unwrap();
        let mut invalid_serialized = invalid_frame.serialize();
        let invalid_len = invalid_frame.serialized_len();
        invalid_serialized[invalid_len - 1] ^= 0xFF; // Corrupt checksum

        // Then, create a valid response for set_max_distance
        let valid_response = Frame::new(0x03, &[0x00]).unwrap();
        let valid_serialized = valid_response.serialize();
        let valid_len = valid_response.serialized_len();

        // Combine both frames
        let mut combined = [0u8; 256];
        combined[..invalid_len].copy_from_slice(&invalid_serialized[..invalid_len]);
        combined[invalid_len..invalid_len + valid_len].copy_from_slice(&valid_serialized[..valid_len]);

        uart.set_rx_data(&combined[..invalid_len + valid_len]);

        let mut driver = LD2410::new(uart, delay);

        // First receive_frame should fail
        let first_result = driver.receive_frame();
        assert_eq!(first_result, Err(Error::InvalidChecksum));

        // Now set_max_distance should work with the valid frame
        let config_result = driver.set_max_distance(600);
        assert!(config_result.is_ok());
    }

    #[test]
    fn test_driver_state_valid_after_multiple_errors() {
        // Test that driver state remains valid after multiple consecutive errors
        let mut uart = MockUart::new();
        let delay = MockDelay;

        // Create multiple invalid frames followed by a valid frame
        let mut buffer = [0u8; 512];
        let mut pos = 0;

        // Invalid frame 1 (corrupted checksum)
        let invalid1 = Frame::new(0x01, &[0x10]).unwrap();
        let mut ser1 = invalid1.serialize();
        let len1 = invalid1.serialized_len();
        ser1[len1 - 1] ^= 0xFF;
        buffer[pos..pos + len1].copy_from_slice(&ser1[..len1]);
        pos += len1;

        // Invalid frame 2 (corrupted checksum)
        let invalid2 = Frame::new(0x02, &[0x20]).unwrap();
        let mut ser2 = invalid2.serialize();
        let len2 = invalid2.serialized_len();
        ser2[len2 - 1] ^= 0xFF;
        buffer[pos..pos + len2].copy_from_slice(&ser2[..len2]);
        pos += len2;

        // Valid frame
        let valid = Frame::new(0x01, &[0x01, 0x64, 0x00, 0x96, 0x00, 0x32, 0x1E]).unwrap();
        let ser_valid = valid.serialize();
        let len_valid = valid.serialized_len();
        buffer[pos..pos + len_valid].copy_from_slice(&ser_valid[..len_valid]);
        pos += len_valid;

        uart.set_rx_data(&buffer[..pos]);

        let mut driver = LD2410::new(uart, delay);

        // First error
        let result1 = driver.receive_frame();
        assert_eq!(result1, Err(Error::InvalidChecksum));

        // Second error
        let result2 = driver.receive_frame();
        assert_eq!(result2, Err(Error::InvalidChecksum));

        // Should still be able to receive valid frame
        let result3 = driver.receive_frame();
        assert!(result3.is_ok());
        let received = result3.unwrap();
        assert_eq!(received.command, 0x01);
    }
}
