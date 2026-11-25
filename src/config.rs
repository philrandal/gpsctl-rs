//! gpsctl.conf settings, read the way gpsctl's getConfigBool()/getConfigInt()/iniparser_getdouble() read them.
//! Keys are lower-case "section:key" (as iniparser names them).  The ini parser is deliberately simple: `[section]`,
//! `key = value`, full-line `;`/`#` comments and trailing ` ;`/` #` comments, optional "double quotes" around a value.

use std::collections::HashMap;

#[derive(Debug, Default, Clone)]
pub struct Config {
    map: HashMap<String, String>,
}

impl Config {
    pub fn new() -> Config { Config::default() }

    pub fn set(&mut self, key: &str, value: &str) {
        self.map.insert(key.to_ascii_lowercase(), value.to_string());
    }

    pub fn parse_ini(text: &str) -> Config {
        let mut c = Config::new();
        let mut section = String::new();
        for raw in text.lines() {
            let line = raw.trim();
            if line.is_empty() || line.starts_with(';') || line.starts_with('#') { continue; }
            if let Some(rest) = line.strip_prefix('[') {
                if let Some(name) = rest.split_once(']') { section = name.0.trim().to_ascii_lowercase(); }
                continue;
            }
            let Some((k, v)) = line.split_once('=') else { continue };
            let mut v = v.trim();
            if let Some(q) = v.strip_prefix('"') {
                v = q.split('"').next().unwrap_or("");
            } else {
                // as iniparser: an unquoted value ends at the first ';' or '#' (no space needed before it)
                if let Some(i) = v.find([';', '#']) { v = v[..i].trim_end(); }
            }
            c.set(&format!("{section}:{}", k.trim()), v);
        }
        c
    }

    pub fn get(&self, key: &str) -> Option<&str> {
        self.map.get(key).map(String::as_str)
    }

    /// getConfigBool(): iniparser's first-character rule; unrecognised text gives `default` (and a warning).
    pub fn bool(&self, key: &str, default: bool, warn: &mut Vec<String>) -> bool {
        let r = crate::cfg_gnss::ini_bool(self.get(key), default);
        if let Some(v) = self.get(key) {
            if crate::cfg_gnss::ini_bool(Some(v), false) != crate::cfg_gnss::ini_bool(Some(v), true) {
                warn.push(format!("{key} = \"{v}\" is not a recognised yes/no value; using the default"));
            }
        }
        r
    }

    /// getConfigInt(): the whole text must be a number (strtol base 0: decimal, 0x hex, leading-0 octal) inside
    /// min..=max, otherwise the default is used (with a warning).
    pub fn int(&self, key: &str, default: i64, min: i64, max: i64, warn: &mut Vec<String>) -> i64 {
        let Some(text) = self.get(key) else { return default };
        match strtol0(text) {
            None => { warn.push(format!("{key} = \"{text}\" is not a whole number; using the default ({default})")); default }
            Some(v) if v < min || v > max => {
                warn.push(format!("{key} = {v} is outside {min}..{max}; using the default ({default})"));
                default
            }
            Some(v) => v,
        }
    }

    /// getConfigDouble(): the whole text must be a finite number inside min..=max, otherwise the default is used (with
    /// a warning).
    pub fn double(&self, key: &str, default: f64, min: f64, max: f64, warn: &mut Vec<String>) -> f64 {
        let Some(text) = self.get(key) else { return default };
        match strtod_whole(text) {
            None => { warn.push(format!("{key} = \"{text}\" is not a number; using the default ({default})")); default }
            Some(v) if v < min || v > max => {
                warn.push(format!("{key} = {v} is outside {min}..{max}; using the default ({default})"));
                default
            }
            Some(v) => v,
        }
    }

    /// getConfigNMEAVersion(): the NMEA version byte for the receiver.  The text must be a number that names a version
    /// u-blox defines (see `cfg_nmea::version_from_number`), otherwise the default byte is used (with a warning).
    pub fn nmea_version(&self, key: &str, default: u8, warn: &mut Vec<String>) -> u8 {
        let Some(text) = self.get(key) else { return default };
        let Some(value) = strtod_whole(text) else {
            warn.push(format!("{key} = \"{text}\" is not a number; using the default (0x{default:02X})"));
            return default;
        };
        match crate::cfg_nmea::version_from_number(value) {
            Some(v) => v,
            None => {
                warn.push(format!("{key} = {text} is not 2.1, 2.3, 4.0, 4.1 or 4.11; using the default (0x{default:02X})"));
                default
            }
        }
    }
}

/// strtol(text, &end, 0) that succeeds only if it consumed the whole (non-empty) text.  Saturates like strtol.
pub fn strtol0(text: &str) -> Option<i64> {
    let b = text.as_bytes();
    let mut i = 0;
    while i < b.len() && matches!(b[i], b' ' | b'\t' | b'\n' | b'\r' | 0x0b | 0x0c) { i += 1; }
    let mut neg = false;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') { neg = b[i] == b'-'; i += 1; }
    let mut radix = 10u32;
    if i + 2 < b.len() + 0 && b[i] == b'0' && (b[i + 1] | 0x20) == b'x' && (b[i + 2] as char).is_ascii_hexdigit() {
        radix = 16;
        i += 2;
    } else if i < b.len() && b[i] == b'0' {
        radix = 8;
    }
    let start = i;
    let mut v: i128 = 0;
    while i < b.len() {
        match (b[i] as char).to_digit(radix) {
            Some(d) => { v = (v * radix as i128 + d as i128).min(i64::MAX as i128 + 1); i += 1; }
            None => break,
        }
    }
    if i == start || i != b.len() { return None; }
    let v = if neg { -v } else { v };
    Some(v.clamp(i64::MIN as i128, i64::MAX as i128) as i64)
}

/// strtod() that succeeds only if it consumed the whole text and the result is finite.  (Decimal forms only; C also
/// accepts hex floats, which a config file has no use for.)
pub fn strtod_whole(text: &str) -> Option<f64> {
    let t = text.trim_start();
    let body = t.strip_prefix(['+', '-']).unwrap_or(t);
    if !body.starts_with(|c: char| c.is_ascii_digit() || c == '.') { return None; }
    t.parse::<f64>().ok().filter(|v| v.is_finite())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strtol_like_c() {
        assert_eq!(strtol0("56"), Some(56));
        assert_eq!(strtol0("-32768"), Some(-32768));
        assert_eq!(strtol0("0x1F"), Some(31));
        assert_eq!(strtol0("010"), Some(8));
        assert_eq!(strtol0("0"), Some(0));
        assert_eq!(strtol0("12abc"), None);
        assert_eq!(strtol0(""), None);
        assert_eq!(strtol0("abc"), None);
        assert_eq!(strtol0("4294967306"), Some(4294967306));
    }

    #[test]
    fn strict_decimals() {
        assert_eq!(strtod_whole("1.15"), Some(1.15));
        assert_eq!(strtod_whole("-.5"), Some(-0.5));
        assert_eq!(strtod_whole("1e2"), Some(100.0));
        for bad in ["abc", "2.5m", "2.5 m", "", "nan", "inf", "1e999", "."] {
            assert_eq!(strtod_whole(bad), None, "{bad}");
        }
    }

    #[test]
    fn getters_follow_gpsctl_with_comments() {
        let c = Config::parse_ini("[t]\ncomment = 56 ; nanoseconds\nhash = 20 # degrees\nunits = 56 nanoseconds\nhex = 0x1F\nrange = 100001\ndcomment = 2.5 ; metres\ndunits = 2.5 m\n");
        let mut w = vec![];
        assert_eq!(c.int("t:comment", -1, -100, 100000, &mut w), 56);
        assert_eq!(c.int("t:hash", -1, -100, 100000, &mut w), 20);
        assert_eq!(c.int("t:hex", -1, -100, 100000, &mut w), 31);
        assert!(w.is_empty());
        assert_eq!(c.int("t:units", -1, -100, 100000, &mut w), -1);
        assert_eq!(c.int("t:range", -1, -100, 100000, &mut w), -1);
        assert_eq!(c.int("t:missing", -1, -100, 100000, &mut w), -1);
        assert_eq!(c.double("t:dcomment", -1.0, -100.0, 100.0, &mut w), 2.5);
        assert_eq!(c.double("t:dunits", -1.0, -100.0, 100.0, &mut w), -1.0);
        assert_eq!(w.len(), 3);
    }

    #[test]
    fn ini_parsing() {
        let c = Config::parse_ini("; c\n[Time Pulse]\nAntenna Cable Delay = 60 ; ns\nx=\"a b\"\ny = 7#c\n");
        assert_eq!(c.get("time pulse:antenna cable delay"), Some("60"));
        assert_eq!(c.get("time pulse:x"), Some("a b"));
        assert_eq!(c.get("time pulse:y"), Some("7"));
    }

    #[test]
    fn shipped_config_reads_cleanly_through_the_tp5_and_nav5_setters() {
        let c = Config::parse_ini(include_str!("../c-golden/gpsctl.conf.shipped"));
        let (mut w, polled36) = (vec![], vec![0u8; 36]);
        crate::cfg_tp5::apply(&[0u8; 32], &c, &mut w).unwrap();
        crate::cfg_nav5_cfg::apply(&polled36, &c, &mut w).unwrap();
        assert!(w.is_empty(), "{w:?}");
        assert_eq!(c.int("time pulse:antenna cable delay", 0, -32768, 32767, &mut w), 56);
    }
}
