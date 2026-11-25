//! The four gpsctl queries (`-Q fix|version|satellites|config`): getting the data from the receiver and printing it,
//! in English or JSON, exactly as gpsctl's doFixQuery / doVersionQuery / doSatelliteQuery / doConfigQuery do.
//! `get_config` is ubxGetConfig(): ten different polls rolled into one structure.

use crate::serial::Transport;
use crate::ubx_io::{Ubx, UbxError};
use crate::{cfg_msg, cfg_prt, mon_ver, nav_pvt, nav_sat};

// poll time limits, as in ublox.c
const NAV_PVT_MAX_MS: u64 = 2000;
const NAV_SAT_MAX_MS: u64 = 2000;
const MON_VER_MAX_MS: u64 = 2000;
const CFG_NAV5_MAX_MS: u64 = 2000;
const CFG_MAX_MS: u64 = 1000; // CFG-ANT, -GNSS, -PRT, -NMEA, -TP5, -RATE, -PMS and -MSG all use 1000

/// An error with its causes, outermost first (gpsctl's slReturn chains): at verbosity 1 only the last one (the root
/// cause) is shown, at 2 and above all of them.
pub type Chain = Vec<String>;

fn chain(outer: &str, cause: UbxError) -> Chain {
    poll_chain(vec![outer.to_string()], cause)
}

/// What C's pollUbx() reports: a poll that gets no answer is "Problem receiving poll response" caused by "timed out while
/// waiting to receive UBX message".
fn poll_chain(mut c: Chain, cause: UbxError) -> Chain {
    if matches!(cause, UbxError::Timeout) {
        c.push("Problem receiving poll response".to_string());
        c.push("timed out while waiting to receive UBX message".to_string());
    } else {
        c.push(cause.to_string());
    }
    c
}

fn size_error(name: &str, got: usize) -> Chain {
    vec![format!("unexpected {name} message body size: {got} bytes")]
}

// ---------------------------------------------------------------------------------------------------------------
// getting the data

pub fn get_fix<T: Transport>(u: &mut Ubx<T>) -> Result<nav_pvt::Fix, Chain> {
    let body = u.poll(nav_pvt::CLASS, nav_pvt::ID, &[], NAV_PVT_MAX_MS).map_err(|e| poll_chain(vec![], e))?;
    // nav_pvt::decode also applies correct_time() when the receiver's time is valid, like ubxGetFix()
    nav_pvt::decode(&body).map_err(|e| size_error("NAV_PVT", e.0))
}

pub fn get_version<T: Transport>(u: &mut Ubx<T>) -> Result<mon_ver::Version, Chain> {
    let body = u.poll(mon_ver::CLASS, mon_ver::ID, &[], MON_VER_MAX_MS).map_err(|e| poll_chain(vec![], e))?;
    Ok(mon_ver::decode(&body))
}

pub fn get_satellites<T: Transport>(u: &mut Ubx<T>) -> Result<Vec<nav_sat::Satellite>, Chain> {
    let body = u.poll(nav_sat::CLASS, nav_sat::ID, &[], NAV_SAT_MAX_MS).map_err(|e| poll_chain(vec![], e))?;
    let mut sats = nav_sat::decode(&body).map_err(|e| size_error("NAV-SAT", e.0))?;
    // used satellites first, then the strongest signals first (the same order gpsctl's qsort() gives, with ties kept
    // in the order the receiver sent them)
    sats.sort_by(|c, d| {
        if c.used && !d.used {
            std::cmp::Ordering::Less
        } else if d.used && !c.used {
            std::cmp::Ordering::Greater
        } else {
            (d.cno as i32).cmp(&(c.cno as i32))
        }
    });
    Ok(sats)
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct GnssRec {
    pub id: u8,
    pub min_chnnls: u8,
    pub max_chnnls: u8,
    pub enabled: bool,
}

/// Everything `-Q config` shows (ubxConfig).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ReceiverConfig {
    pub ant_pwr: bool,
    pub ant_shrt_det: bool,
    pub ant_open_det: bool,
    pub ant_pwr_dwn_on_shrt: bool,
    pub ant_auto_rec: bool,
    pub trk_chnnls: u8,
    pub gnss: Vec<GnssRec>,
    pub model: u8,
    pub mode: u8,
    pub fixed_alt_m: f64,
    pub fixed_alt_var_m2: f64,
    pub min_elev_deg: i8,
    pub p_dop: f64,
    pub t_dop: f64,
    pub p_acc_m: u16,
    pub t_acc_m: u16,
    pub static_hold_thresh_cm_s: u8,
    pub dgnss_timeout_s: u8,
    pub cno_thresh_num_svs: u8,
    pub cno_thresh_db_hz: u8,
    pub static_hold_max_dist_m: u16,
    pub utc_standard: u8,
    pub ant_cable_delay_ns: i16,
    pub rf_group_delay_ns: i16,
    pub freq_period: u32,
    pub freq_period_lock: u32,
    pub pulse_len_ratio: u32,
    pub pulse_len_ratio_lock: u32,
    pub user_config_delay: i32,
    pub time_pulse0_enabled: bool,
    pub lock_gps_freq: bool,
    pub locked_other_set: bool,
    pub is_freq: bool,
    pub is_length: bool,
    pub align_to_tow: bool,
    pub polarity: bool,
    pub grid_utc_tnss: u8,
    pub meas_rate_ms: u16,
    pub nav_rate: u16,
    pub time_ref: u16,
    pub power_setup: u8,
    pub power_interval_secs: u16,
    pub power_on_time_secs: u16,
    pub nmea_enabled: bool,
    pub nmea_version: u8,
    /// GGA, GLL, GSA, GSV, RMC, VTG, GRS, GST, ZDA
    pub nmea_msgs: [bool; 9],
}

fn u16_at(b: &[u8], o: usize) -> u16 { u16::from_le_bytes([b[o], b[o + 1]]) }
fn i16_at(b: &[u8], o: usize) -> i16 { i16::from_le_bytes([b[o], b[o + 1]]) }
fn u32_at(b: &[u8], o: usize) -> u32 { u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]) }
fn i32_at(b: &[u8], o: usize) -> i32 { u32_at(b, o) as i32 }
fn bit(v: u32, n: u32) -> bool { v & (1 << n) != 0 }

/// ubxGetConfig(): poll CFG-ANT, -GNSS, -NAV5, -TP5, -RATE, -PMS, -PRT, -NMEA and CFG-MSG for each NMEA sentence, in
/// that order, stopping at the first failure.
pub fn get_config<T: Transport>(u: &mut Ubx<T>) -> Result<ReceiverConfig, Chain> {
    let mut c = ReceiverConfig::default();

    // antenna
    let b = u.poll(0x06, 0x13, &[], CFG_MAX_MS).map_err(|e| chain("problem getting antenna information from GPS", e))?;
    if b.len() < 4 { return Err(size_error("CFG-ANT", b.len())); }
    let m = u16_at(&b, 0) as u32;
    c.ant_pwr = bit(m, 0);
    c.ant_shrt_det = bit(m, 1);
    c.ant_open_det = bit(m, 2);
    c.ant_pwr_dwn_on_shrt = bit(m, 3);
    c.ant_auto_rec = bit(m, 4);

    // GNSS
    let b = u.poll(0x06, 0x3E, &[], CFG_MAX_MS).map_err(|e| chain("problem getting GNSS information from GPS", e))?;
    if b.len() < 12 { return Err(size_error("CFG-GNSS", b.len())); }
    c.trk_chnnls = b[1];
    // the record count comes from the device: never believe it beyond what the body can hold
    let recs = (b[3] as usize).min((b.len() - 4) / 8);
    if recs == 0 { return Err(vec!["CFG-GNSS message reports no configuration blocks".to_string()]); }
    for i in 0..recs {
        c.gnss.push(GnssRec { id: b[4 + 8 * i], min_chnnls: b[5 + 8 * i], max_chnnls: b[6 + 8 * i], enabled: b[8 + 8 * i] & 1 != 0 });
    }

    // navigation engine
    let b = u.poll(0x06, 0x24, &[], CFG_NAV5_MAX_MS).map_err(|e| chain("problem getting navigation engine information from GPS", e))?;
    if b.len() < 36 { return Err(size_error("CFG-NAV5", b.len())); }
    c.model = b[2];
    c.mode = b[3];
    c.fixed_alt_m = 0.01 * i32_at(&b, 4) as f64;
    c.fixed_alt_var_m2 = 0.0001 * u32_at(&b, 8) as f64; // U4 in the message
    c.min_elev_deg = b[12] as i8;
    c.p_dop = 0.1 * u16_at(&b, 14) as f64;
    c.t_dop = 0.1 * u16_at(&b, 16) as f64;
    c.p_acc_m = u16_at(&b, 18);
    c.t_acc_m = u16_at(&b, 20);
    c.static_hold_thresh_cm_s = b[22];
    c.dgnss_timeout_s = b[23];
    c.cno_thresh_num_svs = b[24];
    c.cno_thresh_db_hz = b[25];
    c.static_hold_max_dist_m = u16_at(&b, 28);
    c.utc_standard = b[30];

    // time pulse 0
    let b = u.poll(0x06, 0x31, &[0], CFG_MAX_MS).map_err(|e| chain("problem getting time pulse information from GPS", e))?;
    if b.len() < 32 { return Err(size_error("CFG-TP5", b.len())); }
    c.ant_cable_delay_ns = i16_at(&b, 4);
    c.rf_group_delay_ns = i16_at(&b, 6);
    c.freq_period = u32_at(&b, 8);
    c.freq_period_lock = u32_at(&b, 12);
    c.pulse_len_ratio = u32_at(&b, 16);
    c.pulse_len_ratio_lock = u32_at(&b, 20);
    c.user_config_delay = i32_at(&b, 24);
    let flags = u32_at(&b, 28);
    c.time_pulse0_enabled = bit(flags, 0);
    c.lock_gps_freq = bit(flags, 1);
    c.locked_other_set = bit(flags, 2);
    c.is_freq = bit(flags, 3);
    c.is_length = bit(flags, 4);
    c.align_to_tow = bit(flags, 5);
    c.polarity = bit(flags, 6);
    c.grid_utc_tnss = ((flags & 0x780) >> 7) as u8;

    // fix rate
    let b = u.poll(0x06, 0x08, &[], CFG_MAX_MS).map_err(|e| chain("problem getting fix rate information from GPS", e))?;
    if b.len() < 6 { return Err(size_error("CFG-RATE", b.len())); }
    c.meas_rate_ms = u16_at(&b, 0);
    c.nav_rate = u16_at(&b, 2);
    c.time_ref = u16_at(&b, 4);

    // power mode
    let b = u.poll(0x06, 0x86, &[], CFG_MAX_MS).map_err(|e| chain("problem getting power mode information from GPS", e))?;
    if b.len() < 8 { return Err(size_error("CFG-PMS", b.len())); }
    c.power_setup = b[1];
    c.power_interval_secs = u16_at(&b, 2);
    c.power_on_time_secs = u16_at(&b, 4);

    // is NMEA on for the UART? (isNmeaOn(): errors are passed up as they are)
    let b = u.poll(cfg_prt::CLASS, cfg_prt::ID, &[cfg_prt::UART_PORT_ID], CFG_MAX_MS).map_err(|e| poll_chain(vec![], e))?;
    if b.len() < 16 { return Err(size_error("CFG-PRT", b.len())); }
    c.nmea_enabled = u16_at(&b, 14) & 2 != 0;

    // NMEA version
    let b = u.poll(0x06, 0x17, &[], CFG_MAX_MS).map_err(|e| chain("problem getting NMEA Version information from GPS", e))?;
    if b.len() < 12 { return Err(size_error("CFG-NMEA", b.len())); }
    c.nmea_version = b[1];

    // which NMEA sentences are on for the UART
    for id in 0..9u8 {
        let b = u.poll(0x06, 0x01, &[0xF0, id], CFG_MAX_MS).map_err(|e| poll_chain(vec![], e))?;
        c.nmea_msgs[id as usize] = cfg_msg::is_enabled(&b).map_err(|e| size_error("CFG-MSG", e.got))?;
    }
    Ok(c)
}

// ---------------------------------------------------------------------------------------------------------------
// names, as in ublox.c

pub fn dynamic_model_name(m: u8) -> &'static str {
    match m {
        0 => "Portable", 2 => "Stationary", 3 => "Pedestrian", 4 => "Automotive", 5 => "Sea",
        6 => "Air (less than 1G acceleration)", 7 => "Air (less than 2G acceleration)",
        8 => "Air (less than 4G acceleration)", 9 => "Watch", 10 => "Bike", _ => "unknown",
    }
}
pub fn fix_mode_name(m: u8) -> &'static str {
    match m { 1 => "2D only", 2 => "3D only", 3 => "Auto 2D/3D", _ => "unknown" }
}
pub fn utc_type_name(m: u8) -> &'static str {
    match m { 0 => "Auto", 3 => "USNO", 5 => "EU", 6 => "GLONASS", 7 => "BeiDou", 8 => "NPLI", 9 => "NICT", _ => "unknown" }
}
pub fn time_grid_name(m: u8) -> &'static str {
    match m { 0 => "UTC", 1 => "GPS", 2 => "GLONASS", 3 => "BeiDou", 4 => "Galileo", _ => "unknown" }
}
pub fn fix_time_ref_name(m: u16) -> &'static str {
    match m { 0 => "UTC", 1 => "GPS", 2 => "GLONASS", 3 => "BeiDou", 4 => "Galileo", _ => "unknown" }
}
pub fn power_mode_name(m: u8) -> &'static str {
    match m {
        0 => "Full", 1 => "Balanced", 2 => "Interval", 3 => "Aggressive (1 Hz)", 4 => "Aggressive (2 Hz)",
        5 => "Aggressive (4 Hz)", 0xFF => "Invalid", _ => "unknown",
    }
}
pub fn signal_quality_name(q: u8) -> &'static str {
    match q {
        0 => "None", 2 => "Acquired", 5 => "Code/carrier locked", 4 => "Code locked", 1 => "Searching", 3 => "Unusable",
        _ => "Unknown",
    }
}
pub fn satellite_health_name(h: u8) -> &'static str {
    match h { 1 => "Ok", 2 => "Bad", _ => "Unknown" }
}
pub fn orbit_source_name(o: u8) -> &'static str {
    match o {
        0 => "None", 2 => "Almanac", 4 => "AssistNow auto", 3 => "AssistNow off", 1 => "Ephemeris", 5 => "Other",
        _ => "Unknown",
    }
}

fn yes_no(b: bool) -> &'static str { if b { "yes" } else { "no" } }
fn on_off(b: bool) -> &'static str { if b { "on" } else { "off" } }

// ---------------------------------------------------------------------------------------------------------------
// JSON, printed like cJSON_PrintUnformatted()

#[derive(Debug, Clone)]
pub enum J {
    Num(f64),
    Bool(bool),
    Str(String),
    Arr(Vec<J>),
    Obj(Vec<(String, J)>),
}

extern "C" {
    fn snprintf(buf: *mut u8, size: usize, format: *const u8, ...) -> i32;
}

/// printf("%1.<prec>g")
fn c_g(d: f64, prec: i32) -> String {
    let fmt = b"%1.*g\0";
    let mut buf = [0u8; 64];
    let n = unsafe { snprintf(buf.as_mut_ptr(), buf.len(), fmt.as_ptr(), prec, d) };
    String::from_utf8_lossy(&buf[..n.max(0) as usize]).into_owned()
}

/// cJSON's number printing: "null" for NaN/infinity, a plain integer when the value is one (within int range), otherwise
/// 15 significant digits, or 17 when 15 would not read back as the same number (to within one part in 2^52, as
/// cJSON's compare_double() says: 0.7000000000000001 prints as 0.7).
pub fn json_number(d: f64) -> String {
    if d.is_nan() || d.is_infinite() {
        return "null".to_string();
    }
    let vi: i32 = if d >= i32::MAX as f64 { i32::MAX } else if d <= i32::MIN as f64 { i32::MIN } else { d as i32 };
    if d == vi as f64 {
        return vi.to_string();
    }
    let s = c_g(d, 15);
    match s.parse::<f64>() {
        Ok(t) if (t - d).abs() <= t.abs().max(d.abs()) * f64::EPSILON => s, // cJSON's compare_double(), not ==
        _ => c_g(d, 17),
    }
}

fn json_string(s: &str, out: &mut String) {
    out.push('"');
    for ch in s.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\u{8}' => out.push_str("\\b"),
            '\u{c}' => out.push_str("\\f"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 32 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
}

impl J {
    pub fn print(&self) -> String {
        let mut out = String::new();
        self.write(&mut out);
        out
    }

    fn write(&self, out: &mut String) {
        match self {
            J::Num(d) => out.push_str(&json_number(*d)),
            J::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
            J::Str(s) => json_string(s, out),
            J::Arr(items) => {
                out.push('[');
                for (i, it) in items.iter().enumerate() {
                    if i > 0 { out.push(','); }
                    it.write(out);
                }
                out.push(']');
            }
            J::Obj(fields) => {
                out.push('{');
                for (i, (k, v)) in fields.iter().enumerate() {
                    if i > 0 { out.push(','); }
                    json_string(k, out);
                    out.push(':');
                    v.write(out);
                }
                out.push('}');
            }
        }
    }
}

fn num<N: Into<f64>>(v: N) -> J { J::Num(v.into()) }
fn obj(fields: Vec<(&str, J)>) -> J { J::Obj(fields.into_iter().map(|(k, v)| (k.to_string(), v)).collect()) }

// ---------------------------------------------------------------------------------------------------------------
// output (what doFixQuery() etc. print): each returns the text including its final newline

pub fn fix_text(f: &nav_pvt::Fix, json: bool) -> String {
    let msl_height_feet = 0.00328084 * f.height_above_sea_level_mm as f64;
    let height_accuracy_feet = 0.00328084 * f.height_accuracy_mm as f64;
    let horizontal_accuracy_feet = 0.00328084 * f.horizontal_accuracy_mm as f64;
    let speed_mph = f.ground_speed_mm_s as f64 * 0.00223694;
    let speed_accuracy_mph = f.ground_speed_accuracy_mm_s as f64 * 0.00223694;

    if json {
        let fix = obj(vec![
            ("latitude_deg", num(f.latitude_deg)),
            ("longitude_deg", num(f.longitude_deg)),
            ("height_above_ellipsoid_mm", num(f.height_above_ellipsoid_mm)),
            ("height_above_mean_sea_level_mm", num(f.height_above_sea_level_mm)),
            ("horizontal_accuracy_mm", num(f.horizontal_accuracy_mm)),
            ("height_accuracy_mm", num(f.height_accuracy_mm)),
            ("ground_speed_mm_sec", num(f.ground_speed_mm_s)),
            ("ground_speed_accuracy_mm_sec", num(f.ground_speed_accuracy_mm_s)),
            ("heading_deg", num(f.heading_deg)),
            ("heading_accuracy_deg", num(f.heading_accuracy_deg)),
            ("valid", J::Bool(f.fix_valid)),
            ("3d", J::Bool(f.fix_is_3d)),
        ]);
        let time = obj(vec![
            ("year", num(f.year)), ("month", num(f.month)), ("day", num(f.day)), ("hour", num(f.hour)),
            ("minute", num(f.minute)), ("second", num(f.second)), ("accuracy_ns", num(f.time_accuracy_ns)),
            ("valid", J::Bool(f.time_valid)), ("resolved", J::Bool(f.time_resolved)),
        ]);
        let root = obj(vec![("fix", fix), ("number_of_satellites_used", num(f.number_of_satellites_used)), ("time", time)]);
        return format!("{}\n", root.print());
    }

    let mut s = String::new();
    if f.time_resolved && f.time_valid && f.date_valid {
        s += &format!(" Time (UTC): {:04}-{:02}-{:02} {:02}:{:02}:{:02} (yyyy-mm-dd hh:mm:ss)\n", f.year, f.month, f.day, f.hour, f.minute, f.second as i32);
    } else {
        s += " Time (UTC): not resolved or not valid\n";
    }
    if f.fix_valid {
        s += &format!("   Latitude: {:12.8} {}\n", f.latitude_deg.abs(), if f.latitude_deg < 0.0 { 'S' } else { 'N' });
        s += &format!("  Longitude: {:12.8} {}\n", f.longitude_deg.abs(), if f.longitude_deg < 0.0 { 'W' } else { 'E' });
        if f.fix_is_3d {
            s += &format!("   Altitude: {:.3} feet\n", msl_height_feet);
        } else {
            s += "   Altitude: unknown\n";
        }
        s += &format!("     Motion: {:.3} mph at {:.3} degrees heading\n", speed_mph, f.heading_deg);
    } else {
        s += "        Fix: invalid\n";
    }
    s += &format!(" Satellites: {} used for computing this fix\n", f.number_of_satellites_used);
    s += &format!(
        "   Accuracy: time ({} ns), height (+/-{:.3} feet), position (+/-{:.3} feet), heading(+/-{:.3} degrees), speed(+/-{:.3} mph)\n",
        f.time_accuracy_ns, height_accuracy_feet, horizontal_accuracy_feet, f.heading_accuracy_deg, speed_accuracy_mph
    );
    s
}

pub fn version_text(v: &mon_ver::Version, json: bool) -> String {
    if json {
        let root = obj(vec![
            ("software_version", J::Str(v.software.clone())),
            ("hardware_version", J::Str(v.hardware.clone())),
            ("extensions", J::Arr(v.extensions.iter().map(|e| J::Str(e.clone())).collect())),
        ]);
        return format!("{}\n", root.print());
    }
    let mut s = format!("Software version: {}\nHardware version: {}\n", v.software, v.hardware);
    for e in &v.extensions {
        s += &format!("       Extension: {e}\n");
    }
    s
}

pub fn satellites_text(sats: &[nav_sat::Satellite], json: bool) -> String {
    if json {
        let list: Vec<J> = sats.iter().map(|s| {
            obj(vec![
                ("gnssID", J::Str(nav_sat::gnss_name(s.gnss_id).to_string())),
                ("satelliteID", num(s.satellite_id)),
                ("CNo", num(s.cno)),
                ("elevation", num(s.elevation)),
                ("azimuth", num(s.azimuth)),
                ("pseudoRangeResidual_meters", num(s.pseudo_range_residual_m)),
                ("signalQuality", J::Str(signal_quality_name(s.signal_quality).to_string())),
                ("used", J::Bool(s.used)),
                ("satelliteHealth", J::Str(satellite_health_name(s.health).to_string())),
                ("diffCorr", J::Bool(s.diff_corr)),
                ("smoothed", J::Bool(s.smoothed)),
                ("orbitSource", J::Str(orbit_source_name(s.orbit_source).to_string())),
                ("haveEphemeris", J::Bool(s.have_ephemeris)),
                ("haveAlmanac", J::Bool(s.have_almanac)),
                ("haveAssistNowOff", J::Bool(s.have_assist_now_off)),
                ("haveAssistNowAuto", J::Bool(s.have_assist_now_auto)),
                ("sbasCorrUsed", J::Bool(s.sbas_corr_used)),
                ("rtcmCorrUsed", J::Bool(s.rtcm_corr_used)),
                ("prCorrUsed", J::Bool(s.pr_corr_used)),
                ("crCorrUsed", J::Bool(s.cr_corr_used)),
                ("doCorrUsed", J::Bool(s.do_corr_used)),
            ])
        }).collect();
        let root = obj(vec![("number_of_satellites", num(sats.len() as f64)), ("satellites", J::Arr(list))]);
        return format!("{}\n", root.print());
    }

    let mut s = String::new();
    s += &format!("\n{:<8}{:>3} {:>3} {:>3} {:>3} {:>7} {:<20}{:<8}{:<15}Flags\n", "GNSS", "ID", "CNo", "El", "Azi", "PRr", "Signal qual", "Sat Hlt", "Orbit Src");
    s += &format!("{:<8}{:>3} {:>3} {:>3} {:>3} {:>7} {:<20}{:<8}{:<15}=====\n", "=======", "===", "===", "===", "===", "=======", "===================", "=======", "==============");
    for x in sats {
        let mut flags = String::new();
        for (on, c) in [
            (x.used, 'u'), (x.diff_corr, 'd'), (x.smoothed, 's'), (x.have_ephemeris, 'e'), (x.have_almanac, 'a'),
            (x.sbas_corr_used, 'S'), (x.rtcm_corr_used, 'R'), (x.pr_corr_used, 'P'), (x.cr_corr_used, 'C'), (x.do_corr_used, 'D'),
        ] {
            if on { flags.push(c); }
        }
        s += &format!("{:<8}{:>3} {:>3} {:>3} {:>3} {:>7.1} {:<20}{:<8}{:<15}{}\n",
            nav_sat::gnss_name(x.gnss_id), x.satellite_id, x.cno, x.elevation, x.azimuth, x.pseudo_range_residual_m,
            signal_quality_name(x.signal_quality), satellite_health_name(x.health), orbit_source_name(x.orbit_source), flags);
    }
    s += "\nFlags:\n  u - used for navigation fix\n  d - differential correction is available\n  s - carrier-smoothed pseudorange used\n  e - ephemeris is available\n  a - almanac is available\n  S - SBAS corrections used\n  R - RTCM corrections used\n  P - pseudorange corrections used\n  C - carrier range corrections used\n  D - range rate (Doppler) corrections used\n";
    s
}

const NMEA_NAMES: [&str; 9] = ["GGA", "GLL", "GSA", "GSV", "RMC", "VTG", "GRS", "GST", "ZDA"];

pub fn config_text(c: &ReceiverConfig, json: bool) -> String {
    if json {
        let ant = obj(vec![
            ("power_on", J::Bool(c.ant_pwr)), ("short_detection", J::Bool(c.ant_shrt_det)), ("open_detection", J::Bool(c.ant_open_det)),
            ("power_down_on_short", J::Bool(c.ant_pwr_dwn_on_shrt)), ("auto_recover_from_short", J::Bool(c.ant_auto_rec)),
        ]);
        let recs: Vec<J> = c.gnss.iter().map(|g| obj(vec![
            ("name", J::Str(nav_sat::gnss_name(g.id).to_string())), ("enabled", J::Bool(g.enabled)),
            ("min_channels", num(g.min_chnnls)), ("max_channels", num(g.max_chnnls)),
        ])).collect();
        // (the C code adds the channel count to the GNSS object before its record array is filled in, but after the
        // array has been attached: so the order of the keys is GNSS_records first, then the channel count)
        let gnss = obj(vec![("GNSS_records", J::Arr(recs)), ("number_of_tracking_channels", num(c.trk_chnnls))]);
        let nav = obj(vec![
            ("dynamic_model", J::Str(dynamic_model_name(c.model).to_string())),
            ("fix_mode", J::Str(fix_mode_name(c.mode).to_string())),
            ("fixed_altitude_2D_meters", num(c.fixed_alt_m)),
            ("fixed_altitude_2D_variance_m2", num(c.fixed_alt_var_m2)),
            ("min_elevation", num(c.min_elev_deg)),
            ("position_dop_mask", num(c.p_dop)),
            ("time_dop_mask", num(c.t_dop)),
            ("position_accuracy_mask", num(c.p_acc_m)),
            ("time_accuracy_mask", num(c.t_acc_m)),
            ("static_hold_threshold_cms", num(c.static_hold_thresh_cm_s)),
            ("dgnss_timeout_secs", num(c.dgnss_timeout_s)),
            ("threshold_satellites_above_cno", num(c.cno_thresh_num_svs)),
            ("cno_threshold_dbhz", num(c.cno_thresh_db_hz)),
            ("static_hold_max_distance_meters", num(c.static_hold_max_dist_m)),
            ("utc_standard", J::Str(utc_type_name(c.utc_standard).to_string())),
        ]);
        let freq_scale = if c.is_freq { 1.0 } else { 0.000001 };
        let len_scale = if c.is_length { 0.000001 } else { 0.000000023283064 };
        let time = obj(vec![
            ("enabled", J::Bool(c.time_pulse0_enabled)), ("is_freq", J::Bool(c.is_freq)), ("is_length", J::Bool(c.is_length)),
            ("lock_on_gps_freq", J::Bool(c.lock_gps_freq)), ("locked_other_set", J::Bool(c.locked_other_set)),
            ("align_to_top_of_second", J::Bool(c.align_to_tow)), ("polarity_rising_edge", J::Bool(c.polarity)),
            ("time_grid", J::Str(time_grid_name(c.grid_utc_tnss).to_string())),
            ("antenna_cable_delay_ns", num(c.ant_cable_delay_ns)),
            ("rf_group_delay_ns", num(c.rf_group_delay_ns)),
            ("user_configured_delay_ns", num(c.user_config_delay)),
            ("freq_period_lock", num(c.freq_period_lock as f64 * freq_scale)),
            ("freq_period", num(c.freq_period as f64 * freq_scale)),
            ("pulse_length_ratio_lock", num(c.pulse_len_ratio_lock as f64 * len_scale)),
            ("pulse_length_ratio", num(c.pulse_len_ratio as f64 * len_scale)),
        ]);
        let rate = obj(vec![
            ("measurement_rate_ms", num(c.meas_rate_ms)), ("measurements_per_fix", num(c.nav_rate)),
            ("time_reference", J::Str(fix_time_ref_name(c.time_ref).to_string())),
        ]);
        let pwr = obj(vec![
            ("power_setup", J::Str(power_mode_name(c.power_setup).to_string())),
            ("period_secs_for_interval", num(c.power_interval_secs)), ("on_time_secs_for_interval", num(c.power_on_time_secs)),
        ]);
        let mut nmea = vec![
            ("enabled", J::Bool(c.nmea_enabled)),
            ("version", num(if c.nmea_version == 0x4B { 4.11 } else { (c.nmea_version / 16) as f64 + (c.nmea_version % 16) as f64 * 0.1 })),
        ];
        for (i, n) in NMEA_NAMES.iter().enumerate() {
            nmea.push((n, J::Bool(c.nmea_msgs[i])));
        }
        let root = obj(vec![
            ("antenna", ant), ("GNSS", gnss), ("navigation", nav), ("time_pulse", time), ("fix_rate", rate),
            ("power_mode", pwr), ("nmea", obj(nmea)),
        ]);
        return format!("{}\n", root.print());
    }

    let mut s = String::new();
    let mut p = |line: String| { s += &line; s.push('\n'); };
    p("U-Blox GPS configuration".to_string());
    p("  Antenna:".to_string());
    p(format!("    Power enabled:                {}", yes_no(c.ant_pwr)));
    p(format!("    Short detection:              {}", yes_no(c.ant_shrt_det)));
    p(format!("    Open detection:               {}", yes_no(c.ant_open_det)));
    p(format!("    Power down on short:          {}", yes_no(c.ant_pwr_dwn_on_shrt)));
    p(format!("    Auto recovery from short:     {}", yes_no(c.ant_auto_rec)));
    p("  GNSS:".to_string());
    p(format!("    Tracking channels:            {}", c.trk_chnnls));
    for g in &c.gnss {
        p(format!("    Type: {}", nav_sat::gnss_name(g.id)));
        p(format!("      Enabled:                    {}", yes_no(g.enabled)));
        p(format!("      Minimum channels:           {}", g.min_chnnls));
        p(format!("      Maximum channels:           {}", g.max_chnnls));
    }
    p("  Navigation engine:".to_string());
    p(format!("    Dynamic model:                {}", dynamic_model_name(c.model)));
    p(format!("    Fix mode:                     {}", fix_mode_name(c.mode)));
    p(format!("    Fixed altitude (2D):          {:.2} meters", c.fixed_alt_m));
    p(format!("    Fixed altitude variance (2D): {:.4} meters^2", c.fixed_alt_var_m2));
    p(format!("    Minimum elevation:            {} degrees", c.min_elev_deg));
    p(format!("    Position DoP mask:            {:.1}", c.p_dop));
    p(format!("    Time DoP mask:                {:.1}", c.t_dop));
    p(format!("    Position accuracy mask:       {} meters", c.p_acc_m));
    p(format!("    Time accuracy mask:           {} meters", c.t_acc_m));
    p(format!("    Static hold threshold:        {} cm/s", c.static_hold_thresh_cm_s));
    p(format!("    Dynamic GNSS timeout:         {} seconds", c.dgnss_timeout_s));
    p(format!("    Threshold above C/No:         {} satellites", c.cno_thresh_num_svs));
    p(format!("    C/No threshold:               {} dBHz", c.cno_thresh_db_hz));
    p(format!("    Static hold max distance:     {} meters", c.static_hold_max_dist_m));
    p(format!("    UTC standard:                 {}", utc_type_name(c.utc_standard)));
    p("  Time pulse:".to_string());
    p(format!("    Time pulse 0 enabled:         {}", yes_no(c.time_pulse0_enabled)));
    p(format!("    Use frequency vs. period:     {}", yes_no(c.is_freq)));
    p(format!("    Use length vs. duty cycle:    {}", yes_no(c.is_length)));
    p(format!("    Lock on GPS frequency:        {}", yes_no(c.lock_gps_freq)));
    p(format!("    Lock use other set:           {}", yes_no(c.locked_other_set)));
    p(format!("    Align to top of second:       {}", yes_no(c.align_to_tow)));
    p(format!("    Polarity rising edge:         {}", yes_no(c.polarity)));
    p(format!("    Time grid:                    {}", time_grid_name(c.grid_utc_tnss)));
    p(format!("    Antenna cable delay:          {} nanoseconds", c.ant_cable_delay_ns));
    p(format!("    RF group delay:               {} nanoseconds", c.rf_group_delay_ns));
    if c.is_freq {
        p(format!("    Locked pulse frequency:       {} Hz", c.freq_period_lock));
        p(format!("    Unlocked pulse frequency:     {} Hz", c.freq_period));
    } else {
        p(format!("    Locked pulse period:          {} microseconds", c.freq_period_lock));
        p(format!("    Unlocked pulse period:        {} microseconds", c.freq_period));
    }
    if c.is_length {
        p(format!("    Locked pulse length:          {} microseconds", c.pulse_len_ratio_lock));
        p(format!("    Unlocked pulse length:        {} microseconds", c.pulse_len_ratio));
    } else {
        p(format!("    Locked pulse duty cycle:      {:.2}%", c.pulse_len_ratio_lock as f64 * 0.000000023283064));
        p(format!("    Unlocked pulse duty cycle:    {:.2}%", c.pulse_len_ratio as f64 * 0.000000023283064));
    }
    p(format!("    User configurable delay:      {} nanoseconds", c.user_config_delay));
    p("  Fix rate:".to_string());
    p(format!("    Measurement rate:             {} milliseconds", c.meas_rate_ms));
    p(format!("    Measurements per fix:         {}", c.nav_rate));
    p(format!("    Time reference:               {}", fix_time_ref_name(c.time_ref)));
    p("  Power mode:".to_string());
    p(format!("    Power setup:                  {}", power_mode_name(c.power_setup)));
    p(format!("    Period (if interval):         {} seconds", c.power_interval_secs));
    p(format!("    On time (if interval):        {} seconds", c.power_on_time_secs));
    p("  NMEA:".to_string());
    p(format!("    Enabled:                      {}", yes_no(c.nmea_enabled)));
    if c.nmea_version == 0x4B {
        p("    Version:                      4.11".to_string());
    } else {
        p(format!("    Version:                      {:x}.{:x}", c.nmea_version / 16, c.nmea_version % 16));
    }
    for (i, n) in NMEA_NAMES.iter().enumerate() {
        p(format!("    {}:                          {}", n, on_off(c.nmea_msgs[i])));
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn json_numbers_print_like_cjson() {
        assert_eq!(json_number(0.0), "0");
        assert_eq!(json_number(-0.0), "0");
        assert_eq!(json_number(42.0), "42");
        assert_eq!(json_number(-7.0), "-7");
        assert_eq!(json_number(0.1), "0.1");
        assert_eq!(json_number(4.1), "4.1");
        assert_eq!(json_number(51.4778123), "51.4778123");
        assert_eq!(json_number(1.0 / 3.0), "0.33333333333333331"); // 15 digits would not read back as the same number
        assert_eq!(json_number(0.25), "0.25");
        // (golden values from the real cJSON in gpsctl: its compare_double() tolerates one epsilon, so these print short)
        assert_eq!(json_number(0.1 + 0.2), "0.3");
        assert_eq!(json_number(7.0 * 0.1), "0.7");
        assert_eq!(json_number(1e-7), "1e-07");
        assert_eq!(json_number(123456789.123456789), "123456789.12345679");
        assert_eq!(json_number(3000000000.0), "3000000000"); // beyond int range, but %1.15g keeps it plain
        assert_eq!(json_number(1e20), "1e+20");
        assert_eq!(json_number(f64::NAN), "null");
        assert_eq!(json_number(f64::INFINITY), "null");
        assert_eq!(json_number(0.000001), "1e-06");
    }

    #[test]
    fn json_strings_are_escaped_like_cjson() {
        let mut s = String::new();
        json_string("a\"b\\c\n\t\u{1}é", &mut s);
        assert_eq!(s, "\"a\\\"b\\\\c\\n\\t\\u0001é\"");
    }

    #[test]
    fn objects_keep_their_order_and_print_without_spaces() {
        let j = obj(vec![("a", num(1)), ("b", J::Arr(vec![J::Bool(true), J::Str("x".into())])), ("c", obj(vec![]))]);
        assert_eq!(j.print(), "{\"a\":1,\"b\":[true,\"x\"],\"c\":{}}");
    }
}
