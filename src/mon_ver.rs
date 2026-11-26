//! UBX MON-VER (class 0x0A, id 0x04): software version (30 bytes), hardware version (10 bytes), then any number of
//! 30-byte extension strings.  Decoding mirrors gpsctl's ubxGetVersion()/copyBodyString(): the body comes from the
//! device, so every string is cut at its first NUL or at the end of its field, and anything missing or short reads as
//! empty rather than failing.  (The ublox crate has a typed MonVer too; this one is checked against the C code.)

use crate::cfg_nmea::frame;

pub const CLASS: u8 = 0x0A;
pub const ID: u8 = 0x04;

pub fn poll_frame() -> Vec<u8> {
    frame(CLASS, ID, &[])
}

#[derive(Debug, PartialEq, Clone)]
pub struct Version {
    pub software: String,
    pub hardware: String,
    pub extensions: Vec<String>,
}

/// u-blox 8 receivers report protocol versions up to 23; anything higher is a newer generation that gpsctl was not
/// written for.
pub const LAST_M8_PROTOCOL_MAJOR: u32 = 23;

impl Version {
    /// The major protocol version in the extensions ("PROTVER=18.00" gives 18), if there is one.
    pub fn protocol_major(&self) -> Option<u32> {
        for ext in &self.extensions {
            let Some(at) = ext.find("PROTVER") else { continue };
            let rest = ext[at + "PROTVER".len()..].trim_start_matches(|c| c == '=' || c == ' ');
            let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
            if !digits.is_empty() { return digits.parse().ok(); }
        }
        None
    }

    /// Says once per run, on stderr, that the receiver is newer than any u-blox 8 (gpsctl is written and tested for
    /// the M8; later generations are configured differently).
    pub fn warn_if_not_m8(&self) {
        use std::sync::atomic::{AtomicBool, Ordering};
        static WARNED: AtomicBool = AtomicBool::new(false);
        if let Some(major) = self.protocol_major() {
            if major > LAST_M8_PROTOCOL_MAJOR && !WARNED.swap(true, Ordering::Relaxed) {
                eprintln!("Warning: this receiver reports protocol version {major}, which is newer than any u-blox 8 (up to {LAST_M8_PROTOCOL_MAJOR}).");
                eprintln!("         gpsctl is written and tested for the u-blox 8, so some settings may not apply to it");
                eprintln!("         (u-blox 9 and later receivers use a different configuration interface).");
            }
        }
    }
}

fn copy_body_string(body: &[u8], offset: usize, max_len: usize) -> String {
    let avail = body.len().saturating_sub(offset);
    let limit = avail.min(max_len);
    let field = &body[offset.min(body.len())..][..limit];
    let len = field.iter().position(|&b| b == 0).unwrap_or(limit);
    String::from_utf8_lossy(&field[..len]).into_owned()
}

pub fn decode(body: &[u8]) -> Version {
    // round up, like the C code, so a partial last extension still counts
    let ne = if body.len() > 40 { (body.len() - 40 + 29) / 30 } else { 0 };
    Version {
        software: copy_body_string(body, 0, 30),
        hardware: copy_body_string(body, 30, 10),
        extensions: (0..ne).map(|i| copy_body_string(body, 40 + 30 * i, 30)).collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn version_with(ext: &[&str]) -> Version {
        Version { software: String::new(), hardware: String::new(), extensions: ext.iter().map(|s| s.to_string()).collect() }
    }

    #[test]
    fn protocol_major_is_read_from_the_extensions() {
        assert_eq!(version_with(&["FWVER=SPG 3.01", "PROTVER=18.00"]).protocol_major(), Some(18));
        assert_eq!(version_with(&["PROTVER=32.00"]).protocol_major(), Some(32));
        assert_eq!(version_with(&["PROTVER 23"]).protocol_major(), Some(23));
        assert_eq!(version_with(&["FWVER=SPG 3.01"]).protocol_major(), None);
        assert_eq!(version_with(&["PROTVER=x"]).protocol_major(), None);
        assert_eq!(version_with(&[]).protocol_major(), None);
        assert!(32 > LAST_M8_PROTOCOL_MAJOR && 23 <= LAST_M8_PROTOCOL_MAJOR);
    }

    const GOLDEN: &str = include_str!("../c-golden/golden_monver.txt");

    fn hex(s: &str) -> Vec<u8> {
        s.split_whitespace().map(|h| u8::from_str_radix(h, 16).unwrap()).collect()
    }

    #[test]
    fn poll_frame_is_the_documented_one() {
        assert_eq!(poll_frame(), [0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34]);
    }

    #[test]
    fn decode_matches_the_c_implementation() {
        let mut n = 0;
        for line in GOLDEN.lines().filter(|l| l.starts_with("ver ")) {
            let parts: Vec<&str> = line.splitn(3, " : ").collect();
            let name = parts[0];
            let want: Vec<&str> = parts[2].split('|').collect(); // count|software|hardware|ext...
            let v = decode(&hex(parts[1]));
            assert_eq!(want[0].parse::<usize>().unwrap(), v.extensions.len(), "{name}: extension count");
            assert_eq!(want[1], v.software, "{name}: software");
            assert_eq!(want[2], v.hardware, "{name}: hardware");
            assert_eq!(&want[3..], v.extensions.iter().map(String::as_str).collect::<Vec<_>>().as_slice(), "{name}: extensions");
            n += 1;
        }
        assert_eq!(n, 8);
    }
}
