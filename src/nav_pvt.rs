//! UBX NAV-PVT (class 0x01, id 0x07): position, velocity, time.  Mirrors gpsctl's ubxGetFix() including its
//! correctTime(): the receiver reports time rounded to the nearest second, and `nano` (the signed rounding error) is
//! added back, borrowing from the minute/hour/day/month/year when that makes the seconds negative.
//! The correction is only applied when the receiver says the time is valid.  Bodies under 92 bytes are rejected.

use crate::cfg_nmea::frame;

pub const CLASS: u8 = 0x01;
pub const ID: u8 = 0x07;
pub const BODY_LEN: usize = 92;

pub fn poll_frame() -> Vec<u8> {
    frame(CLASS, ID, &[])
}

#[derive(Debug, PartialEq, Clone)]
pub struct Fix {
    pub year: i32,
    pub month: i32,
    pub day: i32,
    pub hour: i32,
    pub minute: i32,
    pub second: f64,
    pub nanoseconds_correction: i32,
    pub time_accuracy_ns: u32,
    pub time_resolved: bool,
    pub date_valid: bool,
    pub time_valid: bool,
    pub fix_is_3d: bool,
    pub fix_valid: bool,
    pub number_of_satellites_used: u32,
    pub longitude_deg: f64,
    pub latitude_deg: f64,
    pub height_above_ellipsoid_mm: i32,
    pub height_above_sea_level_mm: i32,
    pub height_accuracy_mm: u32,
    pub horizontal_accuracy_mm: u32,
    pub ground_speed_mm_s: i32,
    pub ground_speed_accuracy_mm_s: u32,
    pub heading_deg: f64,
    pub heading_accuracy_deg: f64,
}

#[derive(Debug, PartialEq)]
pub struct BadBodySize(pub usize);

pub fn is_leap_year(y: u32) -> bool {
    y % 400 == 0 || (y % 4 == 0 && y % 100 != 0)
}

/// Days in a month (1..=12); 0 for anything else, like sl_general.c's daysInMonth().
pub fn days_in_month(year: u32, month: u32) -> u32 {
    const D: [u32; 12] = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
    if month == 0 || month > 12 {
        return 0;
    }
    D[month as usize - 1] + (month == 2 && is_leap_year(year)) as u32
}

/// Add the nanosecond correction to the seconds and, if that went negative, borrow from the larger units.
/// (Like the C code this is wrong across a leap second; and without a valid date there is no day to step back from.)
pub fn correct_time(fix: &mut Fix) {
    fix.second += (fix.nanoseconds_correction as i64) as f64 * 1e-9;
    if fix.second >= 0.0 { return; }
    fix.second += 60.0;
    fix.minute -= 1;
    if fix.minute >= 0 { return; }
    fix.minute += 60;
    fix.hour -= 1;
    if fix.hour >= 0 { return; }
    fix.hour += 24;
    if !fix.date_valid { return; }
    fix.day -= 1;
    if fix.day >= 1 { return; }
    fix.month -= 1;
    if fix.month >= 1 {
        fix.day = days_in_month(fix.year as u32, fix.month as u32) as i32;
        return;
    }
    fix.year -= 1;
    fix.month = 12;
    fix.day = days_in_month(fix.year as u32, fix.month as u32) as i32;
}

pub fn decode(body: &[u8]) -> Result<Fix, BadBodySize> {
    if body.len() < BODY_LEN {
        return Err(BadBodySize(body.len()));
    }
    let u16_ = |o: usize| u16::from_le_bytes([body[o], body[o + 1]]);
    let u32_ = |o: usize| u32::from_le_bytes([body[o], body[o + 1], body[o + 2], body[o + 3]]);
    let i32_ = |o: usize| u32_(o) as i32;
    let bit = |byte: u8, n: u32| byte & (1 << n) != 0;
    let mut fix = Fix {
        year: u16_(4) as i32,
        month: body[6] as i32,
        day: body[7] as i32,
        hour: body[8] as i32,
        minute: body[9] as i32,
        second: body[10] as f64,
        nanoseconds_correction: i32_(16),
        time_accuracy_ns: u32_(12),
        date_valid: bit(body[11], 0),
        time_valid: bit(body[11], 1),
        time_resolved: bit(body[11], 2),
        fix_is_3d: body[20] == 3 || body[20] == 4, // 4 = GNSS + dead reckoning, still a 3D fix
        fix_valid: bit(body[21], 0),
        number_of_satellites_used: body[23] as u32,
        latitude_deg: i32_(28) as f64 * 1e-7,
        longitude_deg: i32_(24) as f64 * 1e-7,
        height_above_ellipsoid_mm: i32_(32),
        height_above_sea_level_mm: i32_(36),
        height_accuracy_mm: u32_(44),
        horizontal_accuracy_mm: u32_(40),
        ground_speed_mm_s: i32_(60),
        ground_speed_accuracy_mm_s: u32_(68),
        heading_deg: i32_(64) as f64 * 1e-5,
        heading_accuracy_deg: u32_(72) as f64 * 1e-5,
    };
    // before the receiver has a valid time the time fields are zero and a correction would turn them into nonsense
    if fix.time_valid {
        correct_time(&mut fix);
    }
    Ok(fix)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::golden_util::cases;

    /// The same line the C harness prints.
    fn line(f: &Fix) -> String {
        format!(
            "{} {} {} {} {} {:.9} {} {} {} {} {} {} {} {} {:.9} {:.9} {} {} {} {} {} {} {:.9} {:.9}",
            f.year, f.month, f.day, f.hour, f.minute, f.second, f.nanoseconds_correction, f.time_accuracy_ns,
            f.time_resolved as u8, f.date_valid as u8, f.time_valid as u8, f.fix_is_3d as u8, f.fix_valid as u8,
            f.number_of_satellites_used, f.longitude_deg, f.latitude_deg, f.height_above_ellipsoid_mm,
            f.height_above_sea_level_mm, f.height_accuracy_mm, f.horizontal_accuracy_mm, f.ground_speed_mm_s,
            f.ground_speed_accuracy_mm_s, f.heading_deg, f.heading_accuracy_deg
        )
    }

    #[test]
    fn fix_type_3_and_4_are_3d_but_not_2_or_5() {
        // 4 is GNSS + dead reckoning (still a 3D fix), as in the C program since 2026-11-25; 2 is 2D, 5 time only
        for (fix_type, want) in [(0u8, false), (1, false), (2, false), (3, true), (4, true), (5, false)] {
            let mut body = vec![0u8; 92];
            body[20] = fix_type;
            assert_eq!(decode(&body).unwrap().fix_is_3d, want, "fixType {fix_type}");
        }
    }

    #[test]
    fn poll_frame_is_the_documented_one() {
        assert_eq!(poll_frame(), [0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19]);
    }

    #[test]
    fn days_in_month_matches_c() {
        assert_eq!(days_in_month(2028, 2), 29);
        assert_eq!(days_in_month(2100, 2), 28);
        assert_eq!(days_in_month(2000, 2), 29);
        assert_eq!(days_in_month(2026, 0), 0);
        assert_eq!(days_in_month(2026, 13), 0);
        assert_eq!(days_in_month(2026, 12), 31);
    }

    #[test]
    fn decode_matches_the_c_implementation() {
        let cs = cases("fix");
        assert_eq!(cs.len(), 17);
        for c in cs {
            match decode(&c.body) {
                Ok(f) => {
                    assert!(c.ok(), "{}: C failed", c.name);
                    assert_eq!(line(&f), c.find("FIX ").unwrap(), "{}", c.name);
                }
                Err(e) => {
                    assert!(!c.ok(), "{}: C succeeded", c.name);
                    assert_eq!(format!("unexpected NAV_PVT message body size: {} bytes", e.0), c.find("ERR: ").unwrap());
                }
            }
        }
    }
}
