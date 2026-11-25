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
