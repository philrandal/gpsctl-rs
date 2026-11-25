//! UBX CFG-MSG (class 0x06, id 0x01) for the NMEA standard messages (class 0xF0).  Mirrors gpsctl's
//! isNMEAmsgEnabled() and ubxEnableNMEAMsg(): poll the message's rates, read or change ONLY the UART1 rate (byte 3),
//! and send the whole polled body back so the other ports keep their rates; send nothing if it is already right.

use crate::cfg_nmea::frame;
use std::fmt;

pub const CLASS: u8 = 0x06;
pub const ID: u8 = 0x01;
/// Class byte of the NMEA standard messages.
pub const NMEA_CLASS: u8 = 0xF0;
/// Reading UART1's rate needs bytes 0..=3; changing it needs the full six-port body (2 + 6 bytes).
pub const MIN_READ_LEN: usize = 4;
pub const MIN_SET_LEN: usize = 8;

/// gpsctl's nmeaMSG enum; the discriminant is the NMEA message id byte.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Nmea { Gga = 0, Gll, Gsa, Gsv, Rmc, Vtg, Grs, Gst, Zda }

impl Nmea {
    pub const ALL: [Nmea; 9] = [Nmea::Gga, Nmea::Gll, Nmea::Gsa, Nmea::Gsv, Nmea::Rmc, Nmea::Vtg, Nmea::Grs, Nmea::Gst, Nmea::Zda];
    pub fn from_id(id: u8) -> Option<Nmea> {
        Nmea::ALL.get(id as usize).copied()
    }
}

#[derive(Debug, PartialEq)]
pub struct TooShort { pub needed: usize, pub got: usize }

impl fmt::Display for TooShort {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "CFG-MSG body too short: {} bytes, need {}", self.got, self.needed)
    }
}

pub fn poll_frame(msg: Nmea) -> Vec<u8> {
    frame(CLASS, ID, &[NMEA_CLASS, msg as u8])
}

/// Is the message on for UART1 (any non-zero rate counts as on, like the C code)?
pub fn is_enabled(body: &[u8]) -> Result<bool, TooShort> {
    if body.len() < MIN_READ_LEN {
        return Err(TooShort { needed: MIN_READ_LEN, got: body.len() });
    }
    Ok(body[3] != 0)
}

/// The frame that sets UART1's rate to 1 (on) or 0 (off) from the polled body, or None when it already is.  Any bytes
/// after the six rates are echoed unchanged.
pub fn enable_frame(polled: &[u8], enable: bool) -> Result<Option<Vec<u8>>, TooShort> {
    if polled.len() < MIN_SET_LEN {
        return Err(TooShort { needed: MIN_SET_LEN, got: polled.len() });
    }
    if (polled[3] != 0) == enable {
        return Ok(None);
    }
    let mut body = polled.to_vec();
    body[3] = enable as u8;
    Ok(Some(frame(CLASS, ID, &body)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::golden_util::cases;

    #[test]
    fn poll_frames_have_the_right_header() {
        let f = poll_frame(Nmea::Rmc);
        assert_eq!(&f[..8], &[0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x04]);
    }

    #[test]
    fn nmea_ids_match_the_c_enum() {
        for (i, m) in Nmea::ALL.iter().enumerate() {
            assert_eq!(*m as usize, i);
            assert_eq!(Nmea::from_id(i as u8), Some(*m));
        }
        assert_eq!(Nmea::from_id(9), None);
    }

    #[test]
    fn query_matches_the_c_implementation() {
        let cs = cases("msgq");
        assert_eq!(cs.len(), 5);
        for c in cs {
            match is_enabled(&c.body) {
                Ok(v) => {
                    assert!(c.ok(), "{}: C failed, Rust succeeded", c.name);
                    assert_eq!(c.find("ENABLED ").unwrap(), if v { "1" } else { "0" }, "{}", c.name);
                }
                Err(_) => assert!(!c.ok(), "{}: C succeeded, Rust failed", c.name),
            }
        }
    }

    #[test]
    fn set_matches_the_c_implementation_byte_for_byte() {
        let cs = cases("msgset");
        assert_eq!(cs.len(), 7);
        for c in cs {
            let enable = c.args[2] == "1";
            let sent = c.find("SENT ").map(crate::golden_util::hex);
            match enable_frame(&c.body, enable) {
                Ok(f) => {
                    assert!(c.ok(), "{}", c.name);
                    assert_eq!(f, sent, "{}", c.name);
                }
                Err(_) => {
                    assert!(!c.ok(), "{}", c.name);
                    assert!(sent.is_none(), "{}", c.name);
                }
            }
        }
    }
}
