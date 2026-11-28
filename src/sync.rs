//! Baud-rate synchronizers and autobaud (gpsctl's asciiBaudRateSynchronizer, nmeaBaudRateSynchronizer,
//! ubxSynchronizer, autoBaudRate and the time budgets that go with them).

use crate::serial::Transport;
use crate::ubx_io::Ubx;
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SyncMethod {
    Ascii,
    Nmea,
    Ubx,
    /// Autobaud's first pass for a UBX receiver (passiveBaudRateSynchronizer() in gpsctl): listens for a valid NMEA
    /// sentence, sending nothing, for at most `PASSIVE_LISTEN_MS`.  Never chosen by the user.
    Passive,
}

/// The longest the passive pass listens at one rate; a receiver with NMEA on sends something every second or so.
pub const PASSIVE_LISTEN_MS: u64 = 1500;

/// MON-VER poll timeout used by the UBX synchronizer (MON_VER_SYNC_MAX_MS in the C code).
pub const MON_VER_SYNC_MS: u64 = 1200;
/// Allowance, in characters at the line rate, for output queued ahead of a synchronizer reply (UBX_SYNC_BACKLOG_CHARS).
pub const UBX_SYNC_BACKLOG_CHARS: u64 = 1024;
/// Autobaud tries these, highest first, down to the minimum.
pub const AUTOBAUD_RATES: [u32; 20] = [921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600, 4800, 2400, 1800, 1200, 600, 300, 200, 150, 134, 110, 75, 50];

/// Time to allow a synchronizer: 250 character times (8N1 = 10 bits each, integer ns like getSpeedInfo) or `min_ms`,
/// whichever is greater.  gpsctl uses 4000 ms in autobaud and 1500 ms when just synchronizing.
pub fn budget_ms(baud: u32, min_ms: u64) -> u64 {
    let ns_bit = 1_000_000_000u64 / baud as u64;
    let ns_char = 10 * ns_bit;
    min_ms.max(ns_char * 250 / 1_000_000)
}

fn ms(v: u64) -> Duration {
    Duration::from_millis(v)
}

/// 25 printable characters in a row (ASCII 32..=127, tab, LF, CR).
pub fn ascii_sync<T: Transport>(u: &mut Ubx<T>, max_ms: u64) -> Result<bool, String> {
    let start = u.t.now();
    u.flush_rx().map_err(|e| format!("could not flush receiver prior to synchronization: {e}"))?;
    let mut count = 0;
    while u.t.now().saturating_sub(start) < ms(max_ms) {
        let left = ms(max_ms).saturating_sub(u.t.now().saturating_sub(start));
        let Some(c) = u.t.read_byte(left)? else { return Ok(false) };
        if (32..=127).contains(&c) || c == 10 || c == 9 || c == 13 { count += 1 } else { count = 0 }
        if count >= 25 {
            return Ok(true);
        }
    }
    Ok(false)
}

fn hex2int(c: u8) -> Option<u8> {
    (c as char).to_digit(16).map(|d| d as u8)
}

/// One complete NMEA sentence with a correct checksum.
pub fn nmea_sync<T: Transport>(u: &mut Ubx<T>, max_ms: u64) -> Result<bool, String> {
    let start = u.t.now();
    u.flush_rx().map_err(|e| format!("could not flush receiver prior to synchronization: {e}"))?;
    let (mut in_line, mut last, mut ck, mut line_len) = (false, [0u8; 3], 0u8, 0usize);
    while u.t.now().saturating_sub(start) < ms(max_ms) {
        let left = ms(max_ms).saturating_sub(u.t.now().saturating_sub(start));
        let Some(c) = u.t.read_byte(left)? else { return Ok(false) };
        if in_line {
            if c == b'\r' || c == b'\n' {
                // a real sentence has a talker and a sentence formatter ("GPGGA", say) before the "*HH"; without this
                // test an empty "$*00" line passes, because its checksum and the "00" are both zero
                if last[0] == b'*' && line_len >= 8 {
                    if let (Some(hi), Some(lo)) = (hex2int(last[1]), hex2int(last[2])) {
                        // the checksum covers everything between '$' and '*', so the '*XX' just seen is taken back out
                        let calc = ck ^ last[0] ^ last[1] ^ last[2];
                        if (hi << 4 | lo) == calc {
                            return Ok(true);
                        }
                    }
                }
                in_line = false;
            } else {
                ck ^= c;
                line_len += 1;
                last = [last[1], last[2], c];
            }
        } else if c == b'$' {
            in_line = true;
            ck = 0;
            line_len = 0;
            last = [0; 3];
        }
    }
    Ok(false)
}

/// How long one UBX synchronizer poll waits at `baud`: the fixed 1200 ms plus the time to receive UBX_SYNC_BACKLOG_CHARS
/// characters, because at slow rates the reply can sit behind queued NMEA output (as ubxSynchronizer() in gpsctl does).
pub fn ubx_poll_ms(baud: u32) -> u64 {
    let ns_char = 10 * (1_000_000_000u64 / baud as u64);
    MON_VER_SYNC_MS + ns_char * UBX_SYNC_BACKLOG_CHARS / 1_000_000
}

/// A MON-VER poll answered, repeated while there is time for another whole poll (and at least two polls' worth of time
/// is allowed at `baud`).  Starting a poll that can't finish only sends more bytes, and at a wrong baud rate each one
/// is a burst of frame errors in the receiver: a u-blox that sees more than 100 disables its UART receiver for minutes.
pub fn ubx_sync<T: Transport>(u: &mut Ubx<T>, max_ms: u64, baud: u32) -> Result<bool, String> {
    let start = u.t.now();
    u.flush_rx().map_err(|e| format!("could not flush receiver prior to synchronization: {e}"))?;
    let poll_ms = ubx_poll_ms(baud);
    let max_ms = max_ms.max(2 * poll_ms);
    while u.t.now().saturating_sub(start) + ms(poll_ms) <= ms(max_ms) {
        if u.poll(0x0A, 0x04, &[], poll_ms).is_ok() {
            return Ok(true);
        }
    }
    Ok(false)
}

pub fn sync<T: Transport>(u: &mut Ubx<T>, method: SyncMethod, max_ms: u64, baud: u32) -> Result<bool, String> {
    match method {
        SyncMethod::Ascii => ascii_sync(u, max_ms),
        SyncMethod::Nmea => nmea_sync(u, max_ms),
        SyncMethod::Ubx => ubx_sync(u, max_ms, baud),
        SyncMethod::Passive => nmea_sync(u, max_ms.min(PASSIVE_LISTEN_MS)),
    }
}

/// Try each rate from 921600 down to `min_baud`; the first that synchronizes is returned and left set on the host.
pub fn auto_baud<T: Transport>(u: &mut Ubx<T>, min_baud: u32, max_baud: u32, first_baud: u32, method: SyncMethod,
                               mut tried: impl FnMut(u32)) -> Result<u32, String> {
    for baud in auto_baud_order(min_baud, max_baud, first_baud) {
        tried(baud);
        u.set_baud(baud).map_err(|e| format!("problem trying to set baud rate to {baud}: {e}"))?;
        u.t.wait_quiet(crate::serial::QUIET_LINE, crate::serial::QUIET_LINE_MAX)
            .map_err(|e| format!("problem waiting for the line to go quiet at {baud} baud: {e}"))?;
        if sync(u, method, budget_ms(baud, 4000), baud)? {
            return Ok(baud);
        }
    }
    Err(format!("could not synchronize at any baud rate from {max_baud} to {min_baud}"))
}

/// The order autoBaudRate() tries rates in: `first_baud` first (if it is above 0, whether or not it is inside the
/// range), then the standard rates from `max_baud` down to `min_baud`, leaving out `first_baud` (already tried).
pub fn auto_baud_order(min_baud: u32, max_baud: u32, first_baud: u32) -> Vec<u32> {
    let mut order = Vec::new();
    if first_baud > 0 { order.push(first_baud); }
    order.extend(AUTOBAUD_RATES.iter().copied().filter(|&b| b <= max_baud && b >= min_baud && b != first_baud));
    order
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::{Kind, Sim};

    const GOLDEN: &str = include_str!("../c-golden/golden_autobaud.txt");

    #[test]
    fn budgets_match_the_c_arithmetic() {
        assert_eq!(budget_ms(921600, 4000), 4000);
        assert_eq!(budget_ms(115200, 1500), 1500);
        assert_eq!(budget_ms(300, 1500), 8333); // nsBit 3_333_333 * 10 * 250 / 1e6
        assert_eq!(budget_ms(50, 4000), 50000); // 25 characters take five seconds at 50 baud; this allows 250
        assert_eq!(budget_ms(110, 4000), 22727);
    }

    #[test]
    fn autobaud_matches_the_c_implementation() {
        let mut n = 0;
        for line in GOLDEN.lines() {
            let p: Vec<&str> = line.splitn(4, " : ").collect();
            let name = p[0].split_whitespace().nth(1).unwrap();
            let a: Vec<&str> = p[1].split_whitespace().collect();
            let method = match a[0] { "ascii" => SyncMethod::Ascii, "nmea" => SyncMethod::Nmea, _ => SyncMethod::Ubx };
            let min: u32 = a[1].parse().unwrap();
            let r: Vec<&str> = p[2].split_whitespace().collect();
            let kind = if r[1] == "nmea" { Kind::Nmea } else { Kind::Ubx };
            let mut u = Ubx::new(Sim::at_baud(r[0].parse().unwrap(), kind));
            let mut tried = vec![];
            let got = match auto_baud(&mut u, min, 921600, 0, method, |b| tried.push(b)) {
                Ok(b) => format!("BAUD {b}"),
                Err(_) => "FAIL".to_string(),
            };
            assert_eq!(got, p[3], "{name}");
            if got.starts_with("BAUD") {
                assert_eq!(u.t.host_baud, got[5..].parse::<u32>().unwrap(), "{name}: host left at the found rate");
            }
            if name == "ascii-nmea-below-min" {
                assert_eq!(tried, vec![921600, 460800, 230400, 115200], "{name}: never goes below the minimum");
            }
            n += 1;
        }
        assert_eq!(n, 7);
    }

    #[test]
    fn auto_baud_tries_highest_first() {
        let mut u = Ubx::new(Sim::at_baud(57600, Kind::Nmea));
        let mut tried = vec![];
        assert_eq!(auto_baud(&mut u, 9600, 921600, 0, SyncMethod::Nmea, |b| tried.push(b)), Ok(57600));
        assert_eq!(tried, vec![921600, 460800, 230400, 115200, 57600]);
    }

    #[test]
    fn ascii_sync_needs_25_in_a_row() {
        // junk resets the count: a matched NMEA receiver passes, a wrong-rate one (high-bit junk) never does
        let mut u = Ubx::new(Sim::at_baud(9600, Kind::Nmea));
        u.t.host_baud = 9600;
        assert_eq!(ascii_sync(&mut u, 1500), Ok(true));
        let mut u = Ubx::new(Sim::at_baud(9600, Kind::Nmea));
        u.t.host_baud = 19200;
        assert_eq!(ascii_sync(&mut u, 1500), Ok(false));
    }

    #[test]
    fn ubx_poll_time_matches_the_c_formula() {
        assert_eq!(ubx_poll_ms(9600), 1200 + 1_041_660 * 1024 / 1_000_000); // 2266 ms
        assert_eq!(ubx_poll_ms(115200), 1200 + 86_800 * 1024 / 1_000_000);
    }

    #[test]
    fn ubx_sync_polls_mon_ver_until_answered() {
        let mut u = Ubx::new(Sim::at_baud(9600, Kind::Ubx));
        u.t.host_baud = 9600;
        assert_eq!(ubx_sync(&mut u, 1500, 9600), Ok(true));
        assert!(u.t.frames_seen.iter().any(|f| (f.0, f.1) == (0x0A, 0x04)));
        let mut u = Ubx::new(Sim::at_baud(9600, Kind::Ubx));
        u.t.host_baud = 4800;
        assert_eq!(ubx_sync(&mut u, 1500, 9600), Ok(false));
    }

    #[test]
    fn at_a_wrong_rate_only_two_polls_are_sent() {
        // 6000 ms at 19200: each poll takes about 2270 ms (1733 plus the line backlog), so after two there is no time
        // for a third whole poll (the old rule, "until the time is up", would have started it)
        let mut u = Ubx::new(Sim::at_baud(9600, Kind::Ubx));
        u.t.host_baud = 19200;
        assert_eq!(ubx_sync(&mut u, 6000, 19200), Ok(false));
        assert_eq!(u.t.bytes_written, 2 * 8, "two 8-byte MON-VER polls");
    }

    #[test]
    fn passive_pass_listens_without_sending() {
        // right rate: valid NMEA is heard, and not a byte was sent
        let mut u = Ubx::new(Sim::at_baud(9600, Kind::Nmea));
        u.t.host_baud = 9600;
        assert_eq!(sync(&mut u, SyncMethod::Passive, 4000, 9600), Ok(true));
        assert!(u.t.frames_seen.is_empty());
        assert_eq!(u.t.bytes_written, 0);
        // autobaud with the passive method finds the rate, from the top, without sending
        let mut u = Ubx::new(Sim::at_baud(9600, Kind::Nmea));
        let mut tried = vec![];
        assert_eq!(auto_baud(&mut u, 9600, 921600, 0, SyncMethod::Passive, |b| tried.push(b)), Ok(9600));
        assert_eq!(tried, vec![921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600]);
        assert_eq!(u.t.bytes_written, 0);
        // a receiver that says nothing (UBX only, NMEA off) is not heard at any rate
        let mut u = Ubx::new(Sim::at_baud(9600, Kind::Ubx));
        assert!(auto_baud(&mut u, 9600, 921600, 0, SyncMethod::Passive, |_| {}).is_err());
        assert_eq!(u.t.bytes_written, 0);
    }

    #[test]
    fn passive_pass_listens_no_longer_than_its_limit() {
        let mut u = Ubx::new(Sim::at_baud(9600, Kind::Ubx));
        u.t.host_baud = 921600;
        assert_eq!(sync(&mut u, SyncMethod::Passive, 4000, 921600), Ok(false));
        assert!(u.t.clock <= Duration::from_millis(PASSIVE_LISTEN_MS + 50), "listened for {:?}", u.t.clock);
    }

    #[test]
    fn auto_baud_order_puts_the_expected_rate_first_and_respects_the_range() {
        assert_eq!(auto_baud_order(9600, 921600, 0)[..4], [921600, 460800, 230400, 115200]);
        assert_eq!(auto_baud_order(9600, 921600, 0).last(), Some(&9600));
        assert_eq!(auto_baud_order(9600, 921600, 115200)[..4], [115200, 921600, 460800, 230400]);
        assert_eq!(auto_baud_order(9600, 921600, 115200).iter().filter(|&&b| b == 115200).count(), 1);
        assert_eq!(auto_baud_order(9600, 115200, 0), vec![115200, 57600, 38400, 19200, 9600]);
        assert_eq!(auto_baud_order(38400, 115200, 9600), vec![9600, 115200, 57600, 38400]); // first rate outside the range
        assert_eq!(auto_baud_order(115200, 115200, 0), vec![115200]);
    }

    #[test]
    fn auto_baud_tries_the_first_rate_first_and_stops_there() {
        let mut u = Ubx::new(Sim::at_baud(115200, Kind::Nmea));
        let mut tried = vec![];
        assert_eq!(auto_baud(&mut u, 9600, 921600, 115200, SyncMethod::Passive, |b| tried.push(b)), Ok(115200));
        assert_eq!(tried, vec![115200]);
        // a miss falls through to the rest of the range
        let mut u = Ubx::new(Sim::at_baud(57600, Kind::Nmea));
        let mut tried = vec![];
        assert_eq!(auto_baud(&mut u, 9600, 921600, 115200, SyncMethod::Passive, |b| tried.push(b)), Ok(57600));
        assert_eq!(tried, vec![115200, 921600, 460800, 230400, 57600]);
        // the maximum is respected, and the failure message names the range
        let mut u = Ubx::new(Sim::at_baud(460800, Kind::Nmea));
        let mut tried = vec![];
        let e = auto_baud(&mut u, 9600, 115200, 0, SyncMethod::Passive, |b| tried.push(b)).unwrap_err();
        assert_eq!(tried, vec![115200, 57600, 38400, 19200, 9600]);
        assert_eq!(e, "could not synchronize at any baud rate from 115200 to 9600");
    }
}
