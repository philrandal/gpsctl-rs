//! Hand-written UBX CFG-NMEA (class 0x06, id 0x17).  The ublox crate (0.10.0) has no typed packet for it, and
//! protocol-18 receivers have no CFG-VALSET to use instead.
//!
//! Mirrors gpsctl: poll the receiver's body, change ONLY byte 1 (the NMEA version) and send the whole body back, so
//! the fields we don't know about (length 4, 12 or 20 depending on firmware) are echoed unchanged; send nothing at
//! all if the version is already what's wanted.  std-only; checked against golden data from the real C code.

use crate::fletcher;

pub const CLASS: u8 = 0x06;
pub const ID: u8 = 0x17;
/// gpsctl's minimum body size for setting the version (byte 1 must exist; see ubxConfigNMEAVersion()).
pub const MIN_BODY_LEN: usize = 4;

/// A complete UBX frame: B5 62 class id len_lo len_hi body ck_a ck_b.
pub fn frame(class: u8, id: u8, body: &[u8]) -> Vec<u8> {
    let len = u16::try_from(body.len()).expect("UBX body longer than 65535 bytes");
    let mut f = vec![0xB5, 0x62, class, id];
    f.extend_from_slice(&len.to_le_bytes());
    f.extend_from_slice(body);
    let (a, b) = fletcher(&f[2..]);
    f.push(a);
    f.push(b);
    f
}

/// The CFG-NMEA poll request (empty body).
pub fn poll_frame() -> Vec<u8> {
    frame(CLASS, ID, &[])
}

#[derive(Debug, PartialEq)]
pub enum FrameError {
    TooShort,
    BadSync,
    LengthMismatch,
    BadChecksum,
}

/// Validate a frame and return (class, id, body).
pub fn parse_frame(f: &[u8]) -> Result<(u8, u8, &[u8]), FrameError> {
    if f.len() < 8 {
        return Err(FrameError::TooShort);
    }
    if f[0] != 0xB5 || f[1] != 0x62 {
        return Err(FrameError::BadSync);
    }
    let len = u16::from_le_bytes([f[4], f[5]]) as usize;
    if f.len() != 8 + len {
        return Err(FrameError::LengthMismatch);
    }
    let (a, b) = fletcher(&f[2..6 + len]);
    if (a, b) != (f[6 + len], f[7 + len]) {
        return Err(FrameError::BadChecksum);
    }
    Ok((f[2], f[3], &f[6..6 + len]))
}

/// Pull the next valid UBX frame out of a receive buffer that also contains NMEA text and partial data.  Bytes before
/// a frame are discarded; a candidate with a bad checksum or an absurd length is skipped by one byte (resync), so a
/// B5 62 that happens to appear inside NMEA text can't wedge the scanner.  Returns None when more bytes are needed.
pub fn extract_frame(buf: &mut Vec<u8>) -> Option<Vec<u8>> {
    const MAX_BODY: usize = 4096;
    loop {
        match buf.windows(2).position(|w| w == [0xB5, 0x62]) {
            None => {
                let keep_last = buf.last() == Some(&0xB5);
                let drop = buf.len() - usize::from(keep_last);
                buf.drain(..drop);
                return None;
            }
            Some(pos) => {
                buf.drain(..pos);
            }
        }
        if buf.len() < 6 {
            return None;
        }
        let len = u16::from_le_bytes([buf[4], buf[5]]) as usize;
        if len > MAX_BODY {
            buf.drain(..1);
            continue;
        }
        if buf.len() < 8 + len {
            return None;
        }
        if parse_frame(&buf[..8 + len]).is_ok() {
            return Some(buf.drain(..8 + len).collect());
        }
        buf.drain(..1); // bad checksum: this wasn't a frame
    }
}

/// The version byte of a body the receiver sent (byte 1: 0x21, 0x23, 0x40, 0x41 ...).
/// Errors, like gpsctl's validateBodySize(), if the body is too short to have one.
pub fn version(body: &[u8]) -> Result<u8, String> {
    if body.len() < MIN_BODY_LEN {
        return Err(format!("unexpected CFG-NMEA message body size: {} bytes", body.len()));
    }
    Ok(body[1])
}

/// What to send to make the receiver use `new_version`, given the body it reported:
/// `Ok(None)` = already set, send nothing; `Ok(Some(frame))` = the full frame to send.
pub fn set_version_frame(receiver_body: &[u8], new_version: u8) -> Result<Option<Vec<u8>>, String> {
    if version(receiver_body)? == new_version {
        return Ok(None);
    }
    let mut body = receiver_body.to_vec();
    body[1] = new_version;
    Ok(Some(frame(CLASS, ID, &body)))
}

/// A configured NMEA version -> the byte for the receiver, as getConfigNMEAVersion() does it.  u-blox defines five
/// values: 0x21 (NMEA 2.1), 0x23 (2.3), 0x40 (4.0), 0x41 (4.1, also written 4.10) and 0x4B (4.11).  The number may be
/// the version as written (2.1, 2.3, 4.0, 4.1, 4.10, 4.11) or the digits of that byte (21, 23, 40, 41, 410, 411).  Work
/// is in hundredths so that 4.11 has room: 2.1 -> 210, 4.1 -> 410, 4.11 -> 411; the digits form is 21 -> 210, 411 -> 411.
/// Anything else is `None`.
pub fn version_from_number(value: f64) -> Option<u8> {
    let digits = if (0.0..10.0).contains(&value) {
        (value * 100.0 + 0.5) as i32
    } else if (10.0..100.0).contains(&value) {
        (value + 0.5) as i32 * 10
    } else if (100.0..1000.0).contains(&value) {
        (value + 0.5) as i32
    } else {
        -1
    };
    match digits {
        210 => Some(0x21),
        230 => Some(0x23),
        400 => Some(0x40),
        410 => Some(0x41),
        411 => Some(0x4B),
        _ => None,
    }
}

/// The version byte actually sent: the configured one, forced to at least 4.1 (0x41) when BeiDou or Galileo is on
/// (0x4B, NMEA 4.11, is higher than 0x41).
pub fn version_byte(version: u8, beidou_or_galileo: bool) -> u8 {
    if beidou_or_galileo && version < 0x41 { 0x41 } else { version }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CONV_CASES: usize = 52; // 26 texts x with/without BeiDou or Galileo

    const GOLDEN: &str = include_str!("../c-golden/golden_nmea.txt");

    fn hex(s: &str) -> Vec<u8> {
        s.split_whitespace().map(|h| u8::from_str_radix(h, 16).unwrap()).collect()
    }

    #[test]
    fn poll_frame_matches_c_and_the_documented_value() {
        assert_eq!(poll_frame(), [0xB5, 0x62, 0x06, 0x17, 0x00, 0x00, 0x1D, 0x5D]);
        let line = GOLDEN.lines().find(|l| l.starts_with("poll")).unwrap();
        assert_eq!(poll_frame(), hex(line.split_once(" : ").unwrap().1));
    }

    #[test]
    fn set_cases_match_the_c_implementation() {
        let mut n = 0;
        for line in GOLDEN.lines().filter(|l| l.starts_with("set ")) {
            let parts: Vec<&str> = line.splitn(3, " : ").collect();
            let head: Vec<&str> = parts[0].split_whitespace().collect(); // set <name> <ver>
            let (name, ver) = (head[1], u8::from_str_radix(head[2], 16).unwrap());
            let (body_hex, outcome) = parts[1].split_once(" : ").map(|(a, b)| (a, b.to_string())).unwrap_or((parts[1], parts[2].to_string()));
            let got = set_version_frame(&hex(body_hex), ver);
            match outcome.trim() {
                "error" => assert!(got.is_err(), "{name}"),
                "unchanged" => assert_eq!(got, Ok(None), "{name}"),
                frame_hex => {
                    let f = got.unwrap().unwrap_or_else(|| panic!("{name}: expected a frame"));
                    assert_eq!(f, hex(frame_hex), "{name}");
                    assert!(parse_frame(&f).is_ok(), "{name}: frame must validate");
                }
            }
            n += 1;
        }
        assert_eq!(n, 6);
    }

    #[test]
    fn version_conversion_matches_the_c_implementation() {
        let mut n = 0;
        for line in GOLDEN.lines().filter(|l| l.starts_with("conv ")) {
            let (left, right) = line.split_once(" : ").unwrap();
            let t: Vec<&str> = left.split_whitespace().collect(); // conv <text> <flag>
            let want = u8::from_str_radix(right.trim(), 16).unwrap();
            // an invalid setting gives the default, 0x41 (the C reads the text, so does this)
            let configured = crate::config::strtod_whole(t[1]).and_then(version_from_number).unwrap_or(0x41);
            assert_eq!(version_byte(configured, t[2] == "1"), want, "{line}");
            n += 1;
        }
        assert_eq!(n, CONV_CASES);
    }

    #[test]
    fn short_and_corrupt_frames_are_rejected() {
        assert!(version(&[0x00, 0x23, 0x08]).is_err());          // 3 bytes: no room for the version
        assert!(set_version_frame(&[], 0x41).is_err());
        let mut f = frame(CLASS, ID, &[0x00, 0x23, 0x08, 0x00]);
        assert_eq!(parse_frame(&f).map(|(c, i, b)| (c, i, b.len())), Ok((CLASS, ID, 4)));
        let last = f.len() - 1;
        f[last] ^= 0xFF;
        assert_eq!(parse_frame(&f), Err(FrameError::BadChecksum));
        assert_eq!(parse_frame(&f[..5]), Err(FrameError::TooShort));
    }

    #[test]
    fn echoes_unknown_bytes_unchanged() {
        // only byte 1 may change, whatever the firmware put in the rest of a 20-byte body
        let body: Vec<u8> = (0u8..20).map(|i| i.wrapping_mul(13).wrapping_add(7)).collect();
        let f = set_version_frame(&body, 0x41).unwrap().unwrap();
        let (_, _, sent) = parse_frame(&f).unwrap();
        for i in 0..20 {
            if i == 1 { assert_eq!(sent[i], 0x41) } else { assert_eq!(sent[i], body[i], "byte {i}") }
        }
    }

    /// A reply captured from a real receiver (u-blox M8, protocol 18, via the poll frame above), followed by the ACK-ACK
    /// the receiver sent for the poll.
    const CAPTURED_REPLY: &str = "b5620617140000410002000000000000000100000000000000007557";
    const CAPTURED_ACK: &str = "b562050102000617254e";

    fn unhex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    #[test]
    fn real_m8_reply_decodes_and_round_trips() {
        let reply = unhex(CAPTURED_REPLY);
        let (class, id, body) = parse_frame(&reply).expect("real frame validates (length and Fletcher checksum)");
        assert_eq!((class, id, body.len()), (CLASS, ID, 20));
        assert_eq!(version(body), Ok(0x41));                    // NMEA 4.1
        assert_eq!(body[3], 0x02);                              // flags
        assert_eq!(body[11], 1);                                // the body's own "version" field: the 20-byte layout

        // already 4.1: nothing to send
        assert_eq!(set_version_frame(body, 0x41), Ok(None));
        // changing to 4.0 changes byte 1 only, and the checksum
        let f = set_version_frame(body, 0x40).unwrap().unwrap();
        let (_, _, sent) = parse_frame(&f).unwrap();
        assert_eq!(sent[1], 0x40);
        assert_eq!(&sent[..1], &body[..1]);
        assert_eq!(&sent[2..], &body[2..]);
        assert_ne!(&f[f.len() - 2..], &reply[reply.len() - 2..]);

        // the ACK-ACK for CFG-NMEA that followed it
        let ack = unhex(CAPTURED_ACK);
        let (c, i, b) = parse_frame(&ack).unwrap();
        assert_eq!((c, i, b), (0x05, 0x01, &[CLASS, ID][..]));
    }

    #[test]
    fn extract_frame_finds_frames_among_nmea_noise_and_resyncs() {
        let good = unhex(CAPTURED_REPLY);
        let mut buf: Vec<u8> = b"$GNGSA,A,3,01,02*3C\r\n".to_vec();
        buf.extend_from_slice(&[0xB5, 0x62, 0x06, 0x17, 0xFF, 0x7F]); // fake header: absurd length, must be skipped
        buf.extend_from_slice(&good[..10]);                              // first half of a real frame...
        assert_eq!(extract_frame(&mut buf), None);                       // ...isn't returned yet
        assert!(!buf.is_empty());                                        // and is kept for later
        buf.extend_from_slice(&good[10..]);
        buf.extend_from_slice(b"$GNRMC,1*00\r\n");
        assert_eq!(extract_frame(&mut buf), Some(good));
        assert_eq!(extract_frame(&mut buf), None);
        assert!(buf.len() < 20, "noise must not accumulate");

        let mut bad = unhex(CAPTURED_REPLY);
        let n = bad.len();
        bad[n - 1] ^= 1;                                                  // corrupt checksum: never returned
        let mut buf2 = bad;
        buf2.extend_from_slice(&unhex(CAPTURED_ACK));
        assert_eq!(extract_frame(&mut buf2), Some(unhex(CAPTURED_ACK))); // resyncs onto the next good frame
    }
}
