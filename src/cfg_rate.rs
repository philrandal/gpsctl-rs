//! UBX CFG-RATE (class 0x06, id 0x08), read-only as in gpsctl's ubxGetConfig(): measurement rate, navigation rate,
//! time reference.  The body comes from the device, so anything under 6 bytes is an error; longer bodies are fine.

use crate::cfg_nmea::frame;

pub const CLASS: u8 = 0x06;
pub const ID: u8 = 0x08;
pub const MIN_BODY_LEN: usize = 6;

pub fn poll_frame() -> Vec<u8> {
    frame(CLASS, ID, &[])
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Rate {
    /// Time between measurements, ms.
    pub meas_rate_ms: u16,
    /// Measurements per navigation solution.
    pub nav_rate: u16,
    /// 0 UTC, 1 GPS, 2 GLONASS, 3 BeiDou, 4 Galileo (raw value kept: unknown values are not rejected, like the C code).
    pub time_ref: u16,
}

pub fn decode(body: &[u8]) -> Option<Rate> {
    if body.len() < MIN_BODY_LEN {
        return None;
    }
    let u = |o: usize| u16::from_le_bytes([body[o], body[o + 1]]);
    Some(Rate { meas_rate_ms: u(0), nav_rate: u(2), time_ref: u(4) })
}

pub fn time_ref_name(t: u16) -> &'static str {
    match t { 0 => "UTC", 1 => "GPS", 2 => "GLONASS", 3 => "BeiDou", 4 => "Galileo", _ => "unknown" }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::golden_util::cases;

    #[test]
    fn poll_frame_is_the_documented_one() {
        assert_eq!(poll_frame(), [0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30]);
    }

    #[test]
    fn decode_matches_the_c_implementation() {
        let cs = cases("rate");
        assert_eq!(cs.len(), 5);
        for c in cs {
            match decode(&c.body) {
                Some(r) => {
                    assert!(c.ok(), "{}", c.name);
                    assert_eq!(c.find("RATE ").unwrap(), format!("{} {} {}", r.meas_rate_ms, r.nav_rate, r.time_ref), "{}", c.name);
                }
                None => assert!(!c.ok(), "{}", c.name),
            }
        }
    }
}
