//! CFG-TP5 (time pulse 0), as gpsctl's ubxConfigTimePulse() changes it: poll, overwrite the configured fields, send back.

use crate::cfg_nmea::frame;
use crate::config::Config;

pub const CLASS: u8 = 0x06;
pub const ID: u8 = 0x31;
pub const MIN_LEN: usize = 32;

pub fn poll_frame() -> Vec<u8> { frame(CLASS, ID, &[0]) }

pub fn apply(polled: &[u8], cfg: &Config, warn: &mut Vec<String>) -> Result<Vec<u8>, String> {
    if polled.len() < MIN_LEN {
        return Err(format!("unexpected CFG-TP5 message body size: {} bytes", polled.len()));
    }
    let mut b = polled.to_vec();
    let mut i = |key: &str, d: i64, lo: i64, hi: i64| cfg.int(key, d, lo, hi, warn);
    // C stores the (int) result into a 16/32-bit field, so a negative value wraps to its two's complement
    b[4..6].copy_from_slice(&(i("time pulse:antenna cable delay", 56, -32768, 32767) as u16).to_le_bytes());
    b[6..8].copy_from_slice(&(i("time pulse:rf group delay", 20, -32768, 32767) as u16).to_le_bytes());
    b[8..12].copy_from_slice(&(i("time pulse:unlocked pulse period", 1_000_000, 0, 2147483647) as u32).to_le_bytes());
    b[16..20].copy_from_slice(&(i("time pulse:unlocked pulse length", 0, 0, 2147483647) as u32).to_le_bytes());
    b[12..16].copy_from_slice(&(i("time pulse:locked pulse period", 1_000_000, 0, 2147483647) as u32).to_le_bytes());
    b[20..24].copy_from_slice(&(i("time pulse:locked pulse length", 500_000, 0, 2147483647) as u32).to_le_bytes());
    b[24..28].copy_from_slice(&(i("time pulse:user configurable delay", 0, i32::MIN as i64, 2147483647) as u32).to_le_bytes());
    b[28..32].copy_from_slice(&0x77u32.to_le_bytes());
    Ok(frame(CLASS, ID, &b))
}
