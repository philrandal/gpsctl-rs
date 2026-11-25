//! UBX CFG-GNSS (class 0x06, id 0x3E) as gpsctl's ubxConfigSatellites() uses it: poll the receiver's body, rewrite the
//! enable bit and min/max channels of each GNSS block it recognises, send the whole body back (bytes we don't touch,
//! including unknown blocks and the sigCfgMask bits of the flags, are echoed unchanged).  std-only; checked against frames
//! produced by the REAL C function running against a fake receiver (see ../c-golden/make_gnss_golden.py).
//!
//! Body layout: msgVer, numTrkChHw, numTrkChUse, numConfigBlocks, then 8-byte blocks:
//! gnssId, resTrkCh (min channels), maxTrkCh, reserved, flags u32 LE (bit 0 = enable).

use crate::config::Config;
use crate::cfg_nmea::{frame, set_version_frame, version_byte};

pub const CLASS: u8 = 0x06;
pub const ID: u8 = 0x3E;

pub fn poll_frame() -> Vec<u8> {
    frame(CLASS, ID, &[])
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Constellation {
    pub enabled: bool,
    pub min_ch: u8,
    pub max_ch: u8,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    pub gps: Constellation,
    pub sbas: Constellation,
    pub galileo: Constellation,
    pub beidou: Constellation,
    pub imes: Constellation,
    pub qzss: Constellation,
    pub glonass: Constellation,
}

/// iniparser_getboolean() as in this repo's iniparser.c: decided by the first character(s) only — y/Y/1/t/T/on = true,
/// n/N/0/f/F/off = false — and ANY other text (e.g. "enabled", "disabled") silently gives the default.
pub fn ini_bool(value: Option<&str>, default: bool) -> bool {
    let Some(v) = value else { return default };
    let b = v.as_bytes();
    let c0 = b.first().copied().unwrap_or(0);
    let c1 = b.get(1).copied().unwrap_or(0);
    match c0 {
        b'y' | b'Y' | b'1' | b't' | b'T' => true,
        b'o' | b'O' if matches!(c1, b'n' | b'N') => true,
        b'n' | b'N' | b'0' | b'f' | b'F' => false,
        b'o' | b'O' if matches!(c1, b'f' | b'F') => false,
        _ => default,
    }
}

impl Settings {
    /// Build the settings from config lookups like "gps:enabled" (lower-case "section:key", as iniparser names them),
    /// using gpsctl's defaults, then apply gpsctl's rule that BeiDou and GLONASS can't both be on alongside GPS/Galileo.
    pub fn from_config(cfg: &Config, warn: &mut Vec<String>) -> Settings {
        // as getConfigBool()/getConfigChannels(): a bad channel count (not a number, or outside 0..=255) gives the
        // default and a warning
        let mut sys = |name: &str, en_default: bool, min: u8, max: u8| -> Constellation {
            let (kmin, kmax) = (format!("{name}:minimum channels"), format!("{name}:maximum channels"));
            Constellation {
                enabled: cfg.bool(&format!("{name}:enabled"), en_default, warn),
                min_ch: cfg.int(&kmin, min as i64, 0, 255, warn) as u8,
                max_ch: cfg.int(&kmax, max as i64, 0, 255, warn) as u8,
            }
        };
        let mut s = Settings {
            gps: sys("gps", true, 8, 16),
            sbas: sys("sbas", false, 1, 3),
            galileo: sys("galileo", true, 4, 8),
            beidou: sys("beidou", false, 8, 16),
            imes: sys("imes", false, 0, 8),
            qzss: sys("qzss", false, 0, 3),
            glonass: sys("glonass", true, 8, 14),
        };
        // only one of BeiDou and GLONASS may be on if GPS or Galileo is on
        if s.beidou.enabled && s.glonass.enabled && (s.gps.enabled || s.galileo.enabled) {
            if cfg.bool("gpsctl:prefer beidou to glonass if both enabled", false, warn) {
                s.glonass.enabled = false;
            } else {
                s.beidou.enabled = false;
            }
        }
        s
    }

    /// gpsctl forces NMEA 4.1 or later when BeiDou or Galileo is enabled.
    pub fn needs_nmea_41(&self) -> bool {
        self.beidou.enabled || self.galileo.enabled
    }

    fn for_id(&self, id: u8) -> Option<&Constellation> {
        Some(match id {
            0 => &self.gps,
            1 => &self.sbas,
            2 => &self.galileo,
            3 => &self.beidou,
            4 => &self.imes,
            5 => &self.qzss,
            6 => &self.glonass,
            _ => return None,
        })
    }
}

/// One GNSS block as the receiver reports it (for display).
#[derive(Debug, PartialEq)]
pub struct Block {
    pub id: u8,
    pub min_ch: u8,
    pub max_ch: u8,
    pub flags: u32,
}

impl Block {
    pub fn name(&self) -> &'static str {
        ["GPS", "SBAS", "Galileo", "BeiDou", "IMES", "QZSS", "GLONASS"].get(self.id as usize).copied().unwrap_or("unknown")
    }
    pub fn enabled(&self) -> bool {
        self.flags & 1 != 0
    }
}

/// The blocks of a receiver body (count clamped to what the body holds, like `apply`).
pub fn blocks(body: &[u8]) -> Result<Vec<Block>, String> {
    if body.len() < 12 {
        return Err(format!("unexpected CFG-GNSS message body size: {} bytes", body.len()));
    }
    let n = (body[3] as usize).min((body.len() - 4) / 8);
    Ok((0..n)
        .map(|i| {
            let o = 4 + 8 * i;
            Block { id: body[o], min_ch: body[o + 1], max_ch: body[o + 2], flags: u32::from_le_bytes(body[o + 4..o + 8].try_into().unwrap()) }
        })
        .collect())
}

/// The body to send back, given the one the receiver reported.  Like the C code, a body too short for even one block is an
/// error, and the block count is clamped to what the body can really hold (the receiver's count isn't trusted).
pub fn apply(receiver_body: &[u8], s: &Settings) -> Result<Vec<u8>, String> {
    if receiver_body.len() < 12 {
        return Err(format!("unexpected CFG-GNSS message body size: {} bytes", receiver_body.len()));
    }
    let mut body = receiver_body.to_vec();
    let blocks = (body[3] as usize).min((body.len() - 4) / 8);
    for i in 0..blocks {
        let o = 4 + 8 * i;
        if let Some(c) = s.for_id(body[o]) {
            body[o + 1] = c.min_ch;
            body[o + 2] = c.max_ch;
            let mut flags = u32::from_le_bytes(body[o + 4..o + 8].try_into().unwrap());
            flags = if c.enabled { flags | 1 } else { flags & !1 };
            body[o + 4..o + 8].copy_from_slice(&flags.to_le_bytes());
        }
    }
    Ok(body)
}

/// The CFG-GNSS frame to send for the given receiver body.
pub fn set_frame(receiver_body: &[u8], s: &Settings) -> Result<Vec<u8>, String> {
    Ok(frame(CLASS, ID, &apply(receiver_body, s)?))
}

/// What ubxConfigSatellites() also does after CFG-GNSS: set the NMEA version it reads from `nmea:version` (default 4.1),
/// forced to at least 4.1 when BeiDou or Galileo is on.  `None` = the receiver already has it; nothing is sent.
pub fn nmea_frame(nmea_version_config: u8, s: &Settings, receiver_nmea_body: &[u8]) -> Result<Option<Vec<u8>>, String> {
    set_version_frame(receiver_nmea_body, version_byte(nmea_version_config, s.needs_nmea_41()))
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOLDEN: &str = include_str!("../c-golden/golden_gnss.txt");

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    #[test]
    fn poll_frame_is_the_documented_one() {
        assert_eq!(poll_frame(), [0xB5, 0x62, 0x06, 0x3E, 0x00, 0x00, 0x44, 0xD2]);
    }

    #[test]
    fn frames_match_the_real_c_function() {
        let mut n = 0;
        for line in GOLDEN.lines().filter(|l| !l.is_empty()) {
            let f: Vec<&str> = line.split('\t').collect();
            let (name, ini, recv, nmea, c_gnss, c_nmea, c_result) = (f[0], f[1].replace("\\n", "\n"), hex(f[2]), hex(f[3]), f[4], f[5], f[6]);
            let cfg = Config::parse_ini(&ini);
            let mut warn = vec![];
            let settings = Settings::from_config(&cfg, &mut warn);
            if name == "chan300-rejected" {
                // C (since patch 0020) warns and uses the default 16, and so does this
                assert_eq!(warn.len(), 1, "{name}");
                assert_eq!(hex(c_gnss)[12], 16, "C used the default: GPS maxTrkCh 16 (frame byte 12)");
            }

            match set_frame(&recv, &settings) {
                Err(_) => {
                    assert!(c_result.starts_with("C-RESULT error"), "{name}: Rust errored but C said {c_result}");
                    assert_eq!((c_gnss, c_nmea), ("none", "none"), "{name}");
                }
                Ok(frame) => {
                    assert!(c_result.ends_with("ok"), "{name}: {c_result}");
                    assert_eq!(hex(c_gnss), frame, "{name}: CFG-GNSS frame");
                    let ver = cfg.nmea_version("nmea:version", 0x41, &mut warn);
                    let want_nmea = nmea_frame(ver, &settings, &nmea).unwrap();
                    match want_nmea {
                        None => assert_eq!(c_nmea, "none", "{name}: C sent a CFG-NMEA frame, Rust says none needed"),
                        Some(f) => assert_eq!(hex(c_nmea), f, "{name}: CFG-NMEA frame"),
                    }
                }
            }
            n += 1;
        }
        assert_eq!(n, 14);
    }

    #[test]
    fn ini_booleans_follow_iniparser() {
        for (v, want) in [("yes", true), ("Y", true), ("1", true), ("true", true), ("on", true), ("ON", true),
                          ("no", false), ("0", false), ("false", false), ("off", false), ("OFF", false)] {
            assert_eq!(ini_bool(Some(v), !want), want, "{v}");
        }
        // anything else silently takes the default, in either direction
        for v in ["enabled", "disabled", "", "maybe", "o"] {
            assert!(ini_bool(Some(v), true));
            assert!(!ini_bool(Some(v), false));
        }
    }

    #[test]
    fn channel_counts_follow_get_config_channels() {
        let cfg = Config::parse_ini("[gps]\nminimum channels = 12\nmaximum channels = 0x20\n[glonass]\nmaximum channels = 300\nminimum channels = abc\n");
        let mut w = vec![];
        let s = Settings::from_config(&cfg, &mut w);
        assert_eq!((s.gps.min_ch, s.gps.max_ch), (12, 32));
        // out of range or not a number: the defaults, with a warning each
        assert_eq!((s.glonass.min_ch, s.glonass.max_ch), (8, 14));
        assert_eq!(w.len(), 2);
    }

    #[test]
    fn unknown_blocks_and_sigcfgmask_are_echoed() {
        let s = Settings::from_config(&Config::new(), &mut vec![]);
        let mut body = vec![0, 22, 22, 2];
        body.extend_from_slice(&[0, 3, 9, 0xAA, 0x00, 0x00, 0x77, 0x01]);  // GPS: enable bit clear, sigCfgMask 0x0177
        body.extend_from_slice(&[9, 1, 2, 0xBB, 0xFE, 0xED, 0xFA, 0xCE]);  // unknown id 9
        let out = apply(&body, &s).unwrap();
        assert_eq!(&out[4..12], &[0, 8, 16, 0xAA, 0x01, 0x00, 0x77, 0x01]); // min/max set, enable bit set, rest kept
        assert_eq!(&out[12..], &body[12..]);                                 // unknown block untouched
        assert_eq!(&out[..4], &body[..4]);
    }

    #[test]
    fn blocks_decode_for_display() {
        let body = hex("00161601" .to_owned().as_str()).into_iter().chain(hex("0008100001000101")).collect::<Vec<u8>>();
        let b = blocks(&body).unwrap();
        assert_eq!(b, vec![Block { id: 0, min_ch: 8, max_ch: 16, flags: 0x0101_0001 }]);
        assert_eq!((b[0].name(), b[0].enabled()), ("GPS", true));
        assert!(blocks(&body[..8]).is_err());
    }
}
