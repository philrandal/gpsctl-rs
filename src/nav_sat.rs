//! UBX NAV-SAT (class 0x01, id 0x35): per-satellite signal data.  Mirrors gpsctl's ubxGetSatellites(): an 8-byte
//! header then 12 bytes per satellite; the satellite count in the header is capped by what the body can really hold
//! (the body comes from the device), and a body under 8 bytes is an error.

use crate::cfg_nmea::frame;

pub const CLASS: u8 = 0x01;
pub const ID: u8 = 0x35;
pub const HEADER_LEN: usize = 8;
pub const SV_LEN: usize = 12;

pub fn poll_frame() -> Vec<u8> {
    frame(CLASS, ID, &[])
}

#[derive(Debug, PartialEq, Clone)]
pub struct Satellite {
    pub gnss_id: u8,
    pub satellite_id: u8,
    pub cno: u8,
    pub elevation: i8,
    pub azimuth: i16,
    pub pseudo_range_residual_m: f64,
    /// 0 none, 1 searching, 2 acquired, 3 unusable, 4 code locked, 5 code and carrier locked (values above 5 clamp to 5)
    pub signal_quality: u8,
    pub used: bool,
    /// 0 unknown, 1 ok, 2 bad; the raw 2-bit field, so 3 is possible from a misbehaving receiver (the C enum has no name for it)
    pub health: u8,
    pub diff_corr: bool,
    pub smoothed: bool,
    /// 0 none, 1 ephemeris, 2 almanac, 3 AssistNow offline, 4 AssistNow autonomous, 5 other (6 and 7 clamp to 5)
    pub orbit_source: u8,
    pub have_ephemeris: bool,
    pub have_almanac: bool,
    pub have_assist_now_off: bool,
    pub have_assist_now_auto: bool,
    pub sbas_corr_used: bool,
    pub rtcm_corr_used: bool,
    pub pr_corr_used: bool,
    pub cr_corr_used: bool,
    pub do_corr_used: bool,
}

#[derive(Debug, PartialEq)]
pub struct BadBodySize(pub usize);

pub fn decode(body: &[u8]) -> Result<Vec<Satellite>, BadBodySize> {
    if body.len() < HEADER_LEN {
        return Err(BadBodySize(body.len()));
    }
    let n = (body[5] as usize).min((body.len() - HEADER_LEN) / SV_LEN);
    Ok((0..n)
        .map(|i| {
            let b = &body[HEADER_LEN + SV_LEN * i..][..SV_LEN];
            let flags = u32::from_le_bytes([b[8], b[9], b[10], b[11]]);
            let bit = |n: u32| flags & (1 << n) != 0;
            Satellite {
                gnss_id: b[0],
                satellite_id: b[1],
                cno: b[2],
                elevation: b[3] as i8,
                azimuth: i16::from_le_bytes([b[4], b[5]]),
                pseudo_range_residual_m: i16::from_le_bytes([b[6], b[7]]) as f64 * 0.1,
                signal_quality: ((flags & 0x07) as u8).min(5),
                used: bit(3),
                health: ((flags >> 4) & 0x03) as u8,
                diff_corr: bit(6),
                smoothed: bit(7),
                orbit_source: (((flags >> 8) & 0x07) as u8).min(5),
                have_ephemeris: bit(11),
                have_almanac: bit(12),
                have_assist_now_off: bit(13),
                have_assist_now_auto: bit(14),
                sbas_corr_used: bit(16),
                rtcm_corr_used: bit(17),
                pr_corr_used: bit(20),
                cr_corr_used: bit(21),
                do_corr_used: bit(22),
            }
        })
        .collect())
}

pub fn gnss_name(id: u8) -> &'static str {
    match id { 0 => "GPS", 1 => "SBAS", 2 => "Galileo", 3 => "BeiDou", 4 => "IMES", 5 => "QZSS", 6 => "GLONASS", _ => "unknown" }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::golden_util::cases;

    fn line(s: &Satellite) -> String {
        let b = |v: bool| v as u8;
        format!(
            "{} {} {} {} {} {:.1} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
            s.gnss_id, s.satellite_id, s.cno, s.elevation, s.azimuth, s.pseudo_range_residual_m, s.signal_quality,
            b(s.used), s.health, b(s.diff_corr), b(s.smoothed), s.orbit_source, b(s.have_ephemeris), b(s.have_almanac),
            b(s.have_assist_now_off), b(s.have_assist_now_auto), b(s.sbas_corr_used), b(s.rtcm_corr_used),
            b(s.pr_corr_used), b(s.cr_corr_used)
        )
    }

    #[test]
    fn poll_frame_is_the_documented_one() {
        assert_eq!(poll_frame(), [0xB5, 0x62, 0x01, 0x35, 0x00, 0x00, 0x36, 0xA3]);
    }

    #[test]
    fn decode_matches_the_c_implementation() {
        let cs = cases("sats");
        assert_eq!(cs.len(), 8);
        for c in cs {
            match decode(&c.body) {
                Ok(sats) => {
                    assert!(c.ok(), "{}", c.name);
                    assert_eq!(c.find("SATS ").unwrap(), sats.len().to_string(), "{}", c.name);
                    let want = c.all("SAT ");
                    let dos = c.all("SATDO ");
                    assert_eq!(want.len(), sats.len(), "{}", c.name);
                    for (i, s) in sats.iter().enumerate() {
                        assert_eq!(line(s), want[i], "{} sat {}", c.name, i);
                        assert_eq!((s.do_corr_used as u8).to_string(), dos[i], "{} sat {} doCorr", c.name, i);
                    }
                }
                Err(e) => {
                    assert!(!c.ok(), "{}", c.name);
                    assert_eq!(format!("unexpected NAV-SAT message body size: {} bytes", e.0), c.find("ERR: ").unwrap());
                }
            }
        }
    }

    #[test]
    fn gnss_names() {
        assert_eq!(gnss_name(0), "GPS");
        assert_eq!(gnss_name(6), "GLONASS");
        assert_eq!(gnss_name(9), "unknown");
    }
}
