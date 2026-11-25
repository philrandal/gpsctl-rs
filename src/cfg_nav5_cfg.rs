//! CFG-NAV5 as gpsctl's ubxConfigNavEngine() changes it: poll, overwrite the configured fields (keeping the bytes it
//! doesn't touch exactly as the receiver reported them), send back.  (`Nav5` in lib.rs builds a body from scratch.)

use crate::cfg_nmea::frame;
use crate::config::Config;
use crate::scale_round;

pub const CLASS: u8 = 0x06;
pub const ID: u8 = 0x24;
pub const MIN_LEN: usize = 36;

pub fn poll_frame() -> Vec<u8> { frame(CLASS, ID, &[]) }

/// getConfigDynamicModel(): 0 (portable) and 2..10 only; 1 is not used.
fn dynamic_model(cfg: &Config, key: &str, default: i64, warn: &mut Vec<String>) -> u8 {
    let m = cfg.int(key, default, 0, 10, warn);
    if m == 1 {
        warn.push(format!("{key} = {m} is not one of the dynamic models u-blox defines; using the default ({default})"));
        return default as u8;
    }
    m as u8
}

/// getConfigUTCStandard(): 0, 3, 5, 6, 7, 8 or 9 only.
fn utc_standard(cfg: &Config, key: &str, default: i64, warn: &mut Vec<String>) -> u8 {
    let u = cfg.int(key, default, 0, 255, warn);
    match u {
        0 | 3 | 5 | 6 | 7 | 8 | 9 => u as u8,
        _ => {
            warn.push(format!("{key} = {u} is not a UTC standard u-blox defines (0, 3, 5, 6, 7, 8 or 9); using the default ({default})"));
            default as u8
        }
    }
}

pub fn apply(polled: &[u8], cfg: &Config, warn: &mut Vec<String>) -> Result<Vec<u8>, String> {
    if polled.len() < MIN_LEN {
        return Err(format!("unexpected CFG-NAV5 message body size: {} bytes", polled.len()));
    }
    let mut b = polled.to_vec();
    let i = |key: &str, d: i64, lo: i64, hi: i64, w: &mut Vec<String>| cfg.int(key, d, lo, hi, w);
    b[0..2].copy_from_slice(&0x05F7u16.to_le_bytes());
    b[2] = dynamic_model(cfg, "navigation engine:dynamic model", 2, warn); // Stationary
    b[3] = i("navigation engine:fix mode", 2, 1, 3, warn) as u8; // Only3D
    b[4..8].copy_from_slice(&(scale_round(cfg.double("navigation engine:fixed altitude (2d)", 0.0, -21474836.48, 21474836.47, warn), 100.0) as u32).to_le_bytes());
    b[8..12].copy_from_slice(&(scale_round(cfg.double("navigation engine:fixed altitude variance (2d)", 0.0, 0.0, 429496.7295, warn), 10000.0) as u32).to_le_bytes());
    b[12] = i("navigation engine:minimum elevation", 20, -90, 90, warn) as u8;
    b[14..16].copy_from_slice(&(scale_round(cfg.double("navigation engine:position dop mask", 10.0, 0.0, 6553.5, warn), 10.0) as u16).to_le_bytes());
    b[16..18].copy_from_slice(&(scale_round(cfg.double("navigation engine:time dop mask", 10.0, 0.0, 6553.5, warn), 10.0) as u16).to_le_bytes());
    b[18..20].copy_from_slice(&(i("navigation engine:position accuracy mask", 40, 0, 65535, warn) as u16).to_le_bytes());
    b[20..22].copy_from_slice(&(i("navigation engine:time accuracy mask", 40, 0, 65535, warn) as u16).to_le_bytes());
    b[22] = i("navigation engine:static hold threshold", 0, 0, 255, warn) as u8;
    b[23] = i("navigation engine:dynamic gnss timeout", 60, 0, 255, warn) as u8;
    b[24] = i("navigation engine:threshold above c/no", 8, 0, 255, warn) as u8;
    b[25] = i("navigation engine:c/no threshold", 20, 0, 255, warn) as u8;
    b[28..30].copy_from_slice(&(i("navigation engine:static hold max distance", 0, 0, 65535, warn) as u16).to_le_bytes());
    b[30] = utc_standard(cfg, "navigation engine:utc standard", 3, warn); // USNO_UTC
    Ok(frame(CLASS, ID, &b))
}
