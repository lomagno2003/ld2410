#![no_std]

//! LD2410 Rust Driver
//!
//! A `no_std` compatible driver for the HLK-LD2410 human presence radar sensor.
//! Communicates over UART (default 256000 baud, 8N1) using a binary framed protocol.
//!
//! ## Protocol overview (very important)
//! - **Sensor report frames** (radar -> host):
//!   - Header: `F4 F3 F2 F1`
//!   - Length: `u16` LE (length of *frame data*)
//!   - Frame data: starts with `type`, then `0xAA`, ends with `0x55 0x00`
//!   - Footer: `F8 F7 F6 F5`
//!
//! - **Command / ACK frames** (host <-> radar):
//!   - Header: `FD FC FB FA`
//!   - Length: `u16` LE (length of *intra-frame data*)
//!   - Intra-frame data: `cmd_word: u16 LE` + payload bytes
//!   - Footer: `04 03 02 01`
//!
//! There is **no CRC16** in the vendor protocol. Integrity is via framing + length + footer.
//!
//! This crate implements:
//! - Robust parsing of continuous **normal-mode** reports (type `0x02`)
//! - Basic command framing + a few common commands (enter/exit config, firmware version,
//!   engineering mode enable/disable, restart, baud set)

use embedded_io::{ErrorType, Read, Write};

/// Error types for LD2410 driver operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// UART read error
    Read(E),
    /// UART write error
    Write(E),
    /// Malformed frame structure (bad header/footer/length/markers)
    InvalidFrame,
    /// Unsupported/unknown report type
    UnknownReportType(u8),
    /// Report content is too short / inconsistent
    InvalidReport,
    /// ACK content is too short / inconsistent
    InvalidAck,
    /// Buffer too small for received frame
    BufferOverflow,
}

/// Normal-mode target state (from vendor docs)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TargetState {
    NoTarget = 0x00,
    Moving = 0x01,
    Stationary = 0x02,
    Both = 0x03,
}

impl TargetState {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x00 => Some(Self::NoTarget),
            0x01 => Some(Self::Moving),
            0x02 => Some(Self::Stationary),
            0x03 => Some(Self::Both),
            _ => None,
        }
    }
}

/// Presence detection data from the LD2410 sensor (normal working mode report type `0x02`)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PresenceData {
    /// Decoded target state
    pub target_state: TargetState,
    /// Moving target distance (cm)
    pub moving_distance_cm: u16,
    /// Moving target energy (0..255)
    pub moving_energy: u8,
    /// Stationary target distance (cm) (vendor table often shows u8; we expose u16 and widen)
    pub still_distance_cm: u16,
    /// Stationary target energy (0..255)
    pub still_energy: u8,
    /// Detection distance (cm) (overall)
    pub detection_distance_cm: u16,
}

impl PresenceData {
    /// Convenience: presence is any target detected (not NoTarget)
    pub fn presence(&self) -> bool {
        self.target_state != TargetState::NoTarget
    }
}

/// Command words (u16) used by LD2410 (subset)
pub mod cmd {
    pub const ENABLE_CONFIG: u16 = 0x00FF;
    pub const END_CONFIG: u16 = 0x00FE;

    pub const SET_MAX_GATES_AND_DURATION: u16 = 0x0060;
    pub const READ_PARAMETERS: u16 = 0x0061;

    pub const ENABLE_ENGINEERING: u16 = 0x0062;
    pub const DISABLE_ENGINEERING: u16 = 0x0063;

    pub const SET_SENSITIVITY: u16 = 0x0064;

    pub const READ_FIRMWARE: u16 = 0x00A0;
    pub const SET_BAUD_RATE: u16 = 0x00A1;
    pub const FACTORY_RESET: u16 = 0x00A2;
    pub const RESTART: u16 = 0x00A3;
}

/// Frame protocol constants
pub mod frame {
    /// Report frames (radar -> host)
    pub const REPORT_HEADER: [u8; 4] = [0xF4, 0xF3, 0xF2, 0xF1];
    pub const REPORT_FOOTER: [u8; 4] = [0xF8, 0xF7, 0xF6, 0xF5];

    /// Command / ACK frames (host <-> radar)
    pub const CMD_HEADER: [u8; 4] = [0xFD, 0xFC, 0xFB, 0xFA];
    pub const CMD_FOOTER: [u8; 4] = [0x04, 0x03, 0x02, 0x01];

    /// Inner markers inside report frame data
    pub const INNER_HEAD: u8 = 0xAA;
    pub const INNER_TAIL: u8 = 0x55;
    pub const INNER_CHECK: u8 = 0x00;

    /// Reasonable upper bound for report frame data length.
    /// Normal reports are small; engineering reports can be bigger but still modest.
    pub const MAX_REPORT_DATA_LEN: usize = 128;

    /// Reasonable upper bound for command intra-frame payload length.
    pub const MAX_CMD_DATA_LEN: usize = 128;
}

/// A parsed command ACK
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ack<'a> {
    /// ACK command word = (sent_cmd | 0x0100)
    pub ack_cmd: u16,
    /// Status code (usually 0x0000 for success)
    pub status: u16,
    /// Remaining payload bytes
    pub payload: &'a [u8],
}

/// LD2410 driver for UART communication
///
/// Generic over UART peripheral implementing embedded-io traits.
///
/// This implementation is **blocking** and reads bytes until a full frame is found.
/// For non-blocking/async usage, wrap UART in a layer that returns quickly or
/// adapt this into a byte-fed state machine.
pub struct LD2410<UART> {
    uart: UART,
    /// scratch buffer used during frame reads
    buf: [u8; 256],
}

impl<UART> LD2410<UART> {
    /// Create a new LD2410 driver
    pub fn new(uart: UART) -> Self {
        Self {
            uart,
            buf: [0u8; 256],
        }
    }

    /// Release the underlying UART
    pub fn free(self) -> UART {
        self.uart
    }
}

impl<UART> LD2410<UART>
where
    UART: Read + Write + ErrorType,
{
    /// Read the next **normal-mode** presence report (type `0x02`) from the continuous stream.
    ///
    /// This will skip engineering reports (type `0x01`) and unknown report types.
    pub fn read_presence(&mut self) -> Result<PresenceData, Error<UART::Error>> {
        loop {
            let frame_data = self.read_report_frame_data()?;
            // frame_data layout: [type][0xAA][...payload...][0x55][0x00]
            if frame_data.len() < 4 {
                continue;
            }

            let r#type = frame_data[0];
            if r#type != 0x02 {
                // Skip non-normal reports for this API
                continue;
            }

            return parse_normal_report(&frame_data).map_err(|_| Error::InvalidReport);
        }
    }

    /// Enter configuration mode (required before most config commands).
    pub fn enter_config_mode(&mut self) -> Result<(), Error<UART::Error>> {
        // cmd 0x00FF, payload u16 LE = 0x0001
        let payload = [0x01, 0x00];
        self.send_command(cmd::ENABLE_CONFIG, &payload)?;
        let ack = self.read_ack()?;
        if ack.status != 0x0000 {
            return Err(Error::InvalidAck);
        }
        Ok(())
    }

    /// Exit configuration mode.
    pub fn exit_config_mode(&mut self) -> Result<(), Error<UART::Error>> {
        self.send_command(cmd::END_CONFIG, &[])?;
        let ack = self.read_ack()?;
        if ack.status != 0x0000 {
            return Err(Error::InvalidAck);
        }
        Ok(())
    }

    /// Enable engineering mode (report type becomes `0x01`).
    pub fn enable_engineering_mode(&mut self) -> Result<(), Error<UART::Error>> {
        self.send_command(cmd::ENABLE_ENGINEERING, &[])?;
        let ack = self.read_ack()?;
        if ack.status != 0x0000 {
            return Err(Error::InvalidAck);
        }
        Ok(())
    }

    /// Disable engineering mode (back to normal mode reports).
    pub fn disable_engineering_mode(&mut self) -> Result<(), Error<UART::Error>> {
        self.send_command(cmd::DISABLE_ENGINEERING, &[])?;
        let ack = self.read_ack()?;
        if ack.status != 0x0000 {
            return Err(Error::InvalidAck);
        }
        Ok(())
    }

    /// Read firmware version payload. The vendor ACK payload includes fields like:
    /// firmware_type(u16), major(u16), minor(u32), etc.
    ///
    /// We return the raw payload bytes after `status`.
    pub fn read_firmware_raw(&mut self) -> Result<[u8; 8], Error<UART::Error>> {
        self.send_command(cmd::READ_FIRMWARE, &[])?;
        let ack = self.read_ack()?;
        if ack.status != 0x0000 {
            return Err(Error::InvalidAck);
        }
        // Expect at least 8 bytes following status in many firmwares (but be conservative).
        if ack.payload.len() < 8 {
            return Err(Error::InvalidAck);
        }
        let mut out = [0u8; 8];
        out.copy_from_slice(&ack.payload[..8]);
        Ok(out)
    }

    /// Restart the module.
    pub fn restart(&mut self) -> Result<(), Error<UART::Error>> {
        self.send_command(cmd::RESTART, &[])?;
        let ack = self.read_ack()?;
        if ack.status != 0x0000 {
            return Err(Error::InvalidAck);
        }
        Ok(())
    }

    /// Send a command frame: [CMD_HEADER][len u16][cmd u16][payload...][CMD_FOOTER]
    pub fn send_command(&mut self, cmd_word: u16, payload: &[u8]) -> Result<(), Error<UART::Error>> {
        if payload.len() > frame::MAX_CMD_DATA_LEN {
            return Err(Error::BufferOverflow);
        }

        // intra_len = cmd_word(2) + payload_len
        let intra_len = 2usize + payload.len();
        let len_le = (intra_len as u16).to_le_bytes();
        let cmd_le = cmd_word.to_le_bytes();

        self.uart.write_all(&frame::CMD_HEADER).map_err(Error::Write)?;
        self.uart.write_all(&len_le).map_err(Error::Write)?;
        self.uart.write_all(&cmd_le).map_err(Error::Write)?;
        if !payload.is_empty() {
            self.uart.write_all(payload).map_err(Error::Write)?;
        }
        self.uart.write_all(&frame::CMD_FOOTER).map_err(Error::Write)?;
        Ok(())
    }

    /// Read a command ACK frame and return parsed `Ack`.
    pub fn read_ack(&mut self) -> Result<Ack<'_>, Error<UART::Error>> {
        let frame_bytes_len = self.read_cmd_frame_into_buf()?;
        let buf = &self.buf[..frame_bytes_len];

        // buf: [hdr4][len2][intra...][ftr4]
        if frame_bytes_len < 4 + 2 + 2 + 2 + 4 {
            // must at least contain cmd_word + status + footer
            return Err(Error::InvalidAck);
        }

        let intra_len = u16::from_le_bytes([buf[4], buf[5]]) as usize;
        let intra_start = 6;
        let intra_end = intra_start + intra_len;

        if intra_end + 4 != frame_bytes_len {
            return Err(Error::InvalidAck);
        }

        let ack_cmd = u16::from_le_bytes([buf[intra_start], buf[intra_start + 1]]);
        if intra_len < 4 {
            return Err(Error::InvalidAck);
        }
        let status = u16::from_le_bytes([buf[intra_start + 2], buf[intra_start + 3]]);
        let payload = &buf[(intra_start + 4)..intra_end];

        Ok(Ack {
            ack_cmd,
            status,
            payload,
        })
    }

    /// Read next report frame, return *frame data* (the bytes covered by the report length).
    ///
    /// Outer: [REPORT_HEADER][len u16][frame_data...][REPORT_FOOTER]
    fn read_report_frame_data(&mut self) -> Result<&[u8], Error<UART::Error>> {
        let total_len = self.read_report_frame_into_buf()?;
        // Layout: [hdr4][len2][frame_data...][ftr4]
        if total_len < 4 + 2 + 4 {
            return Err(Error::InvalidFrame);
        }
        let data_len = u16::from_le_bytes([self.buf[4], self.buf[5]]) as usize;
        let data_start = 6;
        let data_end = data_start + data_len;
        if data_end + 4 != total_len {
            return Err(Error::InvalidFrame);
        }
        Ok(&self.buf[data_start..data_end])
    }

    /// Blocking: scan stream until report header found, then read length, payload, footer.
    fn read_report_frame_into_buf(&mut self) -> Result<usize, Error<UART::Error>> {
        // Find header
        self.scan_for_header(&frame::REPORT_HEADER)?;

        // buf[0..4] already filled with header
        // Read len
        self.read_exact_into(4, 2)?;
        let data_len = u16::from_le_bytes([self.buf[4], self.buf[5]]) as usize;

        if data_len == 0 || data_len > frame::MAX_REPORT_DATA_LEN {
            return Err(Error::InvalidFrame);
        }

        // Read frame_data + footer
        let total_len = 4 + 2 + data_len + 4;
        if total_len > self.buf.len() {
            return Err(Error::BufferOverflow);
        }

        self.read_exact_into(6, data_len + 4)?;

        // Validate footer
        let footer_start = 6 + data_len;
        if &self.buf[footer_start..footer_start + 4] != &frame::REPORT_FOOTER {
            return Err(Error::InvalidFrame);
        }

        Ok(total_len)
    }

    /// Blocking: scan stream until command header found, then read length, intra, footer.
    fn read_cmd_frame_into_buf(&mut self) -> Result<usize, Error<UART::Error>> {
        self.scan_for_header(&frame::CMD_HEADER)?;
        self.read_exact_into(4, 2)?;
        let intra_len = u16::from_le_bytes([self.buf[4], self.buf[5]]) as usize;

        if intra_len == 0 || intra_len > frame::MAX_CMD_DATA_LEN {
            return Err(Error::InvalidFrame);
        }

        let total_len = 4 + 2 + intra_len + 4;
        if total_len > self.buf.len() {
            return Err(Error::BufferOverflow);
        }

        self.read_exact_into(6, intra_len + 4)?;

        let footer_start = 6 + intra_len;
        if &self.buf[footer_start..footer_start + 4] != &frame::CMD_FOOTER {
            return Err(Error::InvalidFrame);
        }

        Ok(total_len)
    }

    /// Scan byte-by-byte until the 4-byte header is matched.
    /// Stores the matched header into `self.buf[0..4]`.
    fn scan_for_header(&mut self, header: &[u8; 4]) -> Result<(), Error<UART::Error>> {
        let mut b = [0u8; 1];

        // Simple rolling match state (0..=3)
        let mut idx = 0usize;
        loop {
            match self.uart.read_exact(&mut b) {
                Ok(()) => {
                    let byte = b[0];

                    if byte == header[idx] {
                        self.buf[idx] = byte;
                        idx += 1;
                        if idx == 4 {
                            return Ok(());
                        }
                    } else {
                        // restart match; but if this byte equals header[0], keep it as first
                        if byte == header[0] {
                            self.buf[0] = byte;
                            idx = 1;
                        } else {
                            idx = 0;
                        }
                    }
                }
                Err(embedded_io::ReadExactError::UnexpectedEof) => return Err(Error::InvalidFrame),
                Err(embedded_io::ReadExactError::Other(e)) => return Err(Error::Read(e)),
            }
        }
    }

    /// Read exactly `len` bytes into `self.buf[offset..offset+len]`.
    fn read_exact_into(&mut self, offset: usize, len: usize) -> Result<(), Error<UART::Error>> {
        let end = offset + len;
        if end > self.buf.len() {
            return Err(Error::BufferOverflow);
        }
        match self.uart.read_exact(&mut self.buf[offset..end]) {
            Ok(()) => Ok(()),
            Err(embedded_io::ReadExactError::UnexpectedEof) => Err(Error::InvalidFrame),
            Err(embedded_io::ReadExactError::Other(e)) => Err(Error::Read(e)),
        }
    }
}

/// Parse normal report (type `0x02`) from full *frame_data* (includes type/head/tail/check).
///
/// This function is defensive: some firmwares/document tables are inconsistent on whether
/// still/detection distances are u8 or u16, and there may be 0 padding.
/// We locate the tail (`0x55`) and parse relative to it.
fn parse_normal_report(frame_data: &[u8]) -> Result<PresenceData, ()> {
    // Must be at least: type, head, ..., tail, check
    if frame_data.len() < 6 {
        return Err(());
    }
    let r#type = frame_data[0];
    if r#type != 0x02 {
        return Err(());
    }
    if frame_data[1] != frame::INNER_HEAD {
        return Err(());
    }
    // Tail should be second-to-last, check last
    if frame_data[frame_data.len() - 2] != frame::INNER_TAIL {
        return Err(());
    }
    if frame_data[frame_data.len() - 1] != frame::INNER_CHECK {
        return Err(());
    }

    // Payload between head and tail
    let payload = &frame_data[2..frame_data.len() - 2];

    // Minimum expected core fields (common interpretation):
    // target_state(u8),
    // moving_distance(u16),
    // moving_energy(u8),
    // still_distance(u8 or u16),
    // still_energy(u8),
    // detection_distance(u8 or u16),
    // plus optional padding
    if payload.len() < 1 + 2 + 1 + 1 + 1 + 1 {
        return Err(());
    }

    let target_state = TargetState::from_u8(payload[0]).ok_or(())?;
    let moving_distance_cm = u16::from_le_bytes([payload[1], payload[2]]);
    let moving_energy = payload[3];

    // The remaining bytes are ambiguous across docs/implementations, so parse flexibly.
    // Strategy:
    // - Assume still_energy is a single byte somewhere after still_distance.
    // - Assume detection_distance may be u8 or u16.
    //
    // We try the common compact form first:
    // [still_dist(u8)][still_energy(u8)][detect_dist(u8)] (then optional padding)
    //
    // If payload is longer, we still take these positions, and allow detect_dist to widen
    // if there is a clear u16.
    let mut idx = 4;

    // still distance: prefer u8, widen to u16
    if idx >= payload.len() {
        return Err(());
    }
    let still_distance_cm = payload[idx] as u16;
    idx += 1;

    if idx >= payload.len() {
        return Err(());
    }
    let still_energy = payload[idx];
    idx += 1;

    if idx >= payload.len() {
        return Err(());
    }

    // detection distance: if we have at least 2 bytes before padding, and the next byte is 0x00 often,
    // treat it as u16 LE. Otherwise treat as u8.
    let detection_distance_cm = if (idx + 1) < payload.len() && payload[idx + 1] == 0x00 {
        let d = u16::from_le_bytes([payload[idx], payload[idx + 1]]);
        d
    } else {
        payload[idx] as u16
    };

    Ok(PresenceData {
        target_state,
        moving_distance_cm,
        moving_energy,
        still_distance_cm,
        still_energy,
        detection_distance_cm,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_normal_report_example() {
        // Vendor example (outer length 0x000D):
        // Frame data: 02 AA 02 51 00 00 00 00 3B 00 00 55 00
        // We pass only frame_data here.
        let frame_data: [u8; 13] = [
            0x02, 0xAA, 0x02, 0x51, 0x00, 0x00, 0x00, 0x00, 0x3B, 0x00, 0x00, 0x55, 0x00,
        ];

        let p = super::parse_normal_report(&frame_data).unwrap();
        assert_eq!(p.target_state, TargetState::Stationary);
        assert_eq!(p.moving_distance_cm, 0x0051);
        assert_eq!(p.moving_energy, 0x00);
        assert_eq!(p.still_distance_cm, 0x00);
        assert_eq!(p.still_energy, 0x00);
        // detection distance seen as u16 because next byte is 0x00 => 0x003B
        assert_eq!(p.detection_distance_cm, 0x003B);
        assert!(p.presence());
    }
}
