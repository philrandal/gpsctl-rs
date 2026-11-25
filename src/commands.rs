//! The poll-and-send sequences behind gpsctl's configuration commands (ubxSaveConfig, ubxReset, ubxSetNMEAData,
//! ubxEnableNMEAMsg, ubxConfigNMEAVersion, ubxConfigSatellites, ubxConfigTimePulse, ubxConfigNavEngine,
//! ubxConfigForTiming), over any `Transport`.  Each returns `Err(message)` where the C function returns an error;
//! `warn` collects what gpsctl would print as a "Warning:" (bad config values, non-fatal failures).

use crate::cfg_gnss;
use crate::cfg_msg::{self, Nmea};
use crate::cfg_nav5_cfg;
use crate::cfg_nmea;
use crate::cfg_prt;
use crate::cfg_tp5;
use crate::cfg_cfg;
use crate::config::Config;
use crate::serial::Transport;
use crate::ubx_io::{Ubx, UbxError};
use std::time::Duration;

/// Poll timeouts, as the CFG_*_MAX_MS constants in ublox.c.
pub const CFG_MAX_MS: u64 = 1000;
pub const CFG_NAV5_MAX_MS: u64 = 2000;

type R = Result<(), String>;

/// How long to wait for a serial device to come back after it has gone away, and how often to look
/// (SERIAL_RETURN_MS / SERIAL_LOOK_MS in gpsctl.c).
pub const SERIAL_RETURN_MS: u64 = 5000;
pub const SERIAL_LOOK_MS: u64 = 200;

/// actionRecover(): run `action`; when it fails because the port is no longer usable (hung up: the device went away),
/// wait for the device to come back, reopen it, and run the action once more.  Any other failure is returned as it is,
/// because reopening the port would not help it.  The receiver may have reset while the device was away, so the caller
/// must not assume anything about its state, such as a UBX synchronization, from before.
pub fn with_recovery<T: Transport, F: FnMut(&mut Ubx<T>) -> R>(u: &mut Ubx<T>, mut action: F) -> R {
    let first = match action(u) {
        Ok(()) => return Ok(()),
        Err(e) => e,
    };
    if !u.hung_up() {
        return Err(first);
    }
    u.reopen(Duration::from_millis(SERIAL_RETURN_MS), Duration::from_millis(SERIAL_LOOK_MS))
        .map_err(|e| format!("serial port is not usable and could not be reopened: {e}"))?;
    action(u)
}

fn poll<T: Transport>(u: &mut Ubx<T>, class: u8, id: u8, body: &[u8], ms: u64, what: &str) -> Result<Vec<u8>, String> {
    u.poll(class, id, body, ms).map_err(|e| format!("{what}: {e}"))
}

fn send_acked<T: Transport>(u: &mut Ubx<T>, frame: &[u8], what: &str) -> R {
    // frames carry their own header, so split it back into class/id/body for the sender
    let (class, id, body) = (frame[2], frame[3], &frame[6..frame.len() - 2]);
    u.send_acked(class, id, body).map_err(|e: UbxError| match e {
        // C's sendUbxAckedMsg(): "Problem receiving ACK message" caused by the timeout
        UbxError::Timeout => format!("{what}: Problem receiving ACK message: timed out while waiting to receive UBX message"),
        e => format!("{what}: {e}"),
    })
}

fn too_short(what: &str, needed: usize, got: usize) -> String {
    format!("unexpected {what} message body size: {got} bytes (need {needed})")
}

/// ubxSaveConfig(): CFG-CFG, wait for the ACK.
pub fn save_config<T: Transport>(u: &mut Ubx<T>) -> R {
    send_acked(u, &cfg_cfg::save_frame(), "failed to save GPS configuration")
}

/// `save_config` with another section mask (only for experiments: gpsctl itself always saves 0x041B).
pub fn save_config_mask<T: Transport>(u: &mut Ubx<T>, mask: u32) -> R {
    send_acked(u, &cfg_cfg::save_frame_mask(mask), "failed to save GPS configuration")
}

/// ubxReset(): cold start; the receiver never acknowledges it, so there is nothing to wait for.
pub fn reset<T: Transport>(u: &mut Ubx<T>) -> R {
    let f = cfg_cfg::reset_frame();
    u.send(f[2], f[3], &f[6..f.len() - 2]).map_err(|e| e.to_string())
}

/// ubxSetNMEAData(): switch NMEA output on the UART on or off (bit 1 of the CFG-PRT output protocols).
pub fn set_nmea_data<T: Transport>(u: &mut Ubx<T>, on: bool) -> R {
    set_nmea_data_say(u, on, &mut |_, _| {})
}

/// `set_nmea_data`, also giving what C prints (`say(lowest verbosity that prints it, text)`).
pub fn set_nmea_data_say<T: Transport>(u: &mut Ubx<T>, on: bool, say: &mut dyn FnMut(u8, String)) -> R {
    let onoff = if on { "on" } else { "off" };
    say(3, format!("Querying to see if GPS UART already has NMEA data turned {onoff}..."));
    let cur = poll(u, 0x06, 0x00, &[cfg_prt::UART_PORT_ID], CFG_MAX_MS, "CFG-PRT poll")?;
    let state = cfg_prt::nmea_out(&cur).map_err(|t| too_short("CFG-PRT", t.needed, t.got))?;
    if state == on {
        say(3, format!("GPS UART already has NMEA data turned {onoff}..."));
        return Ok(());
    }
    say(3, "Polling GPS UART configuration...".to_string());
    let mut body = poll(u, 0x06, 0x00, &[cfg_prt::UART_PORT_ID], CFG_MAX_MS, "CFG-PRT poll")?;
    if body.len() < cfg_prt::MIN_BAUD_LEN {
        return Err(too_short("CFG-PRT", cfg_prt::MIN_BAUD_LEN, body.len()));
    }
    let flags = u16::from_le_bytes([body[14], body[15]]);
    let flags = if on { flags | 0x0002 } else { flags & !0x0002 };
    body[14..16].copy_from_slice(&flags.to_le_bytes());
    say(3, format!("Sending new configuration to GPS with NMEA data turned {onoff} for the UART..."));
    match u.send_acked(0x06, 0x00, &body) {
        Ok(_) => {
            say(1, format!("Successfully set new configuration to turn NMEA data {onoff} for GPS UART..."));
            Ok(())
        }
        Err(e) => {
            say(1, format!("Failed to change GPS UART configuration to turn NMEA data {onoff}..."));
            Err(e.to_string())
        }
    }
}

/// ubxEnableNMEAMsg(): poll the message's rates, change only UART1's, send (nothing if already right).
pub fn enable_nmea_msg<T: Transport>(u: &mut Ubx<T>, msg: Nmea, enable: bool) -> R {
    let polled = poll(u, 0x06, 0x01, &[cfg_msg::NMEA_CLASS, msg as u8], CFG_MAX_MS, "failed to get current NMEA MSG configuration")?;
    match cfg_msg::enable_frame(&polled, enable).map_err(|t| too_short("CFG-MSG", t.needed, t.got))? {
        None => Ok(()),
        Some(f) => send_acked(u, &f, "failed to enable /disable NMEA MSG"),
    }
}

/// ubxConfigNMEAVersion(): set byte 1 of CFG-NMEA to `version` (nothing sent if it already is).
pub fn config_nmea_version<T: Transport>(u: &mut Ubx<T>, version: u8) -> R {
    let polled = poll(u, 0x06, 0x17, &[], CFG_MAX_MS, "problem getting NMEA Version information from GPS")?;
    match cfg_nmea::set_version_frame(&polled, version)? {
        None => Ok(()),
        Some(f) => send_acked(u, &f, "problem sending NMEA Version configuration to GPS"),
    }
}

/// ubxConfigSatellites(): CFG-GNSS from the config, then the NMEA version it implies (a failure there is a warning).
pub fn config_satellites<T: Transport>(u: &mut Ubx<T>, cfg: &Config, warn: &mut Vec<String>) -> R {
    config_satellites_say(u, cfg, warn, &mut |_, _| {})
}

/// After the ACK of a CFG-GNSS u-blox wants 0.5 s before the next command.
pub const GNSS_SETTLE_MS: u64 = 500;
/// After asking for a restart: let the receiver go down, then give it up to this long in all to answer again.
pub const GNSS_RESTART_WAIT_MS: u64 = 500;
pub const GNSS_RESTART_MAX_MS: u64 = 10000;

/// ubxHardwareRestartAndWait(): CFG-RST (hot start, resetMode 0x04 = hardware reset after a controlled shutdown, never
/// ACKed), wait, and resynchronize.  A hardware reset reloads the saved configuration and discards the current one.
fn hardware_restart_and_wait<T: Transport>(u: &mut Ubx<T>, say: &mut dyn FnMut(u8, String)) -> R {
    u.send(0x06, 0x04, &[0x00, 0x00, 0x04, 0x00]).map_err(|e| format!("problem restarting the GPS: {e}"))?;
    u.t.pause(Duration::from_millis(GNSS_RESTART_WAIT_MS));
    let baud = u.t.baud();
    match crate::sync::ubx_sync(u, GNSS_RESTART_MAX_MS, baud) {
        Err(e) => Err(format!("problem waiting for the GPS to restart: {e}")),
        Ok(false) => Err("the GPS did not answer after it was restarted (try the -a option)".to_string()),
        Ok(true) => { say(2, "GPS restarted...".to_string()); Ok(()) }
    }
}

/// ubxRestartAfterGnssChange(): u-blox requires a hardware reset after CFG-GNSS with Galileo enabled.  A hardware reset
/// reloads the saved configuration, so save first (0x041B), then restart.
fn restart_after_gnss_change<T: Transport>(u: &mut Ubx<T>, say: &mut dyn FnMut(u8, String)) -> R {
    say(1, "Saving the GPS configuration and restarting the GPS, as u-blox requires after changing GNSS with Galileo enabled...".to_string());
    save_config(u).map_err(|e| format!("problem saving the GPS configuration before restarting the GPS: {e}"))?;
    say(1, "Saved GPS configuration".to_string());
    hardware_restart_and_wait(u, say)
}

/// The receiver's default UART rate, which a factory reset puts it back to.
pub const FACTORY_BAUD: u32 = 9600;

/// ubxFactoryReset(): CFG-CFG with every section cleared (clearMask 0xFFFF: deletes the saved configuration) and loaded
/// (loadMask 0xFFFF: the defaults become current), in BBR, flash, EEPROM and SPI flash (deviceMask 0x17).  The ACK is
/// not relied on (it can come at either rate); the host follows the receiver to 9600 baud, and after the receiver
/// answers there it is restarted from its now-empty saved configuration.
pub fn factory_reset<T: Transport>(u: &mut Ubx<T>, say: &mut dyn FnMut(u8, String)) -> R {
    say(1, format!("Restoring the GPS to its factory defaults (its serial port will be at {FACTORY_BAUD} baud)..."));
    let mut body = Vec::with_capacity(13);
    body.extend_from_slice(&0xFFFFu32.to_le_bytes()); // clear mask: all sections
    body.extend_from_slice(&0u32.to_le_bytes());      // save mask: nothing
    body.extend_from_slice(&0xFFFFu32.to_le_bytes()); // load mask: all sections
    body.push(0x17);
    u.send(0x06, 0x09, &body).map_err(|e| format!("problem sending the factory reset to the GPS: {e}"))?;

    u.t.pause(Duration::from_millis(GNSS_RESTART_WAIT_MS));
    u.t.set_baud(FACTORY_BAUD).map_err(|e| format!("problem changing the host baud rate after the factory reset: {e}"))?;
    u.t.wait_quiet(crate::serial::QUIET_LINE, crate::serial::QUIET_LINE_MAX)
        .map_err(|e| format!("problem waiting for the line to go quiet after the factory reset: {e}"))?;
    match crate::sync::ubx_sync(u, GNSS_RESTART_MAX_MS, FACTORY_BAUD) {
        Err(e) => return Err(format!("problem waiting for the GPS after the factory reset: {e}")),
        Ok(false) => return Err(format!("the GPS did not answer at {FACTORY_BAUD} baud after the factory reset (try the -a option)")),
        Ok(true) => say(2, format!("GPS answers at {FACTORY_BAUD} baud...")),
    }
    hardware_restart_and_wait(u, say).map_err(|e| format!("problem restarting the GPS after the factory reset: {e}"))
}

/// `config_satellites`, also giving what C prints.  CFG-GNSS is sent only if it changes something (every CFG-GNSS makes
/// the receiver restart its GNSS engine); if it was sent, wait 0.5 s, and with Galileo enabled save and restart.
pub fn config_satellites_say<T: Transport>(u: &mut Ubx<T>, cfg: &Config, warn: &mut Vec<String>, say: &mut dyn FnMut(u8, String)) -> R {
    let settings = cfg_gnss::Settings::from_config(cfg, warn);
    let polled = poll(u, 0x06, 0x3E, &[], CFG_MAX_MS, "problem getting GNSS information from GPS")?;
    let f = cfg_gnss::set_frame(&polled, &settings)?;
    if f[6..f.len() - 2] == polled[..] {
        say(2, "GNSS configuration is already as requested...".to_string());
    } else {
        send_acked(u, &f, "problem sending GNSS configuration to GPS")?;
        u.t.pause(Duration::from_millis(GNSS_SETTLE_MS));
        if settings.galileo.enabled {
            restart_after_gnss_change(u, say)
                .map_err(|e| format!("problem restarting the GPS after changing its GNSS configuration: {e}"))?;
        }
    }

    let ver = cfg.nmea_version("nmea:version", 0x41, warn);
    let byte = cfg_nmea::version_byte(ver, settings.needs_nmea_41());
    if let Err(e) = config_nmea_version(u, byte) {
        warn.push(format!("problem configuring NMEA version: {e}"));
    }
    Ok(())
}

/// ubxConfigTimePulse()
pub fn config_time_pulse<T: Transport>(u: &mut Ubx<T>, cfg: &Config, warn: &mut Vec<String>) -> R {
    let polled = poll(u, 0x06, 0x31, &[0], CFG_MAX_MS, "problem getting time pulse information from GPS")?;
    let f = cfg_tp5::apply(&polled, cfg, warn)?;
    send_acked(u, &f, "problem sending time pulse configuration to GPS")
}

/// ubxConfigNavEngine()
pub fn config_nav_engine<T: Transport>(u: &mut Ubx<T>, cfg: &Config, warn: &mut Vec<String>) -> R {
    let polled = poll(u, 0x06, 0x24, &[], CFG_NAV5_MAX_MS, "problem getting navigation engine information from GPS")?;
    let f = cfg_nav5_cfg::apply(&polled, cfg, warn)?;
    send_acked(u, &f, "problem sending navigation engine configuration to GPS")
}

/// ubxConfigForTiming(): satellites, time pulse and navigation engine (each fatal), then each NMEA message on or off as
/// the config says (a failure is only a warning).
pub fn config_for_timing<T: Transport>(u: &mut Ubx<T>, cfg: &Config, warn: &mut Vec<String>) -> R {
    config_for_timing_say(u, cfg, warn, &mut |_, _| {})
}

pub fn config_for_timing_say<T: Transport>(u: &mut Ubx<T>, cfg: &Config, warn: &mut Vec<String>, say: &mut dyn FnMut(u8, String)) -> R {
    config_satellites_say(u, cfg, warn, say).map_err(|e| format!("problem configuring GNSS satellites for timing: {e}"))?;
    config_time_pulse(u, cfg, warn).map_err(|e| format!("problem configuring time pulse for timing: {e}"))?;
    config_nav_engine(u, cfg, warn).map_err(|e| format!("problem configuring navigation engine for timing: {e}"))?;

    // (message, config key, default)
    let msgs = [
        (Nmea::Gga, "nmea:gga", false), (Nmea::Gll, "nmea:gll", false), (Nmea::Gsa, "nmea:gsa", false),
        (Nmea::Gsv, "nmea:gsv", false), (Nmea::Rmc, "nmea:rmc", true), (Nmea::Vtg, "nmea:vtg", false),
        (Nmea::Grs, "nmea:grs", false), (Nmea::Gst, "nmea:gst", false), (Nmea::Zda, "nmea:zda", true),
    ];
    for (msg, key, default) in msgs {
        let enable = cfg.bool(key, default, warn);
        if let Err(e) = enable_nmea_msg(u, msg, enable) {
            warn.push(format!("problem configuring NMEA message output for timing: {e}"));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::FakeAny;
    use std::collections::HashMap;

    const GOLDEN: &str = include_str!("../c-golden/golden_cmd.txt");

    /// Runs the command named in a golden line against a `FakeAny`; Ok/Err plus the frames the receiver was sent.
    fn run_case(args: &[&str], cfg: &Config, bodies: HashMap<(u8, u8), Vec<u8>>, stateful: bool, warn: &mut Vec<String>) -> (R, Vec<Vec<u8>>) {
        let mut u = Ubx::new(FakeAny::new(bodies, stateful));
        let r = match args[0] {
            "save" => save_config(&mut u),
            "reset" => reset(&mut u),
            "nmeadata" => set_nmea_data(&mut u, args[1] == "1"),
            "msg" => enable_nmea_msg(&mut u, Nmea::from_id(args[1].parse().unwrap()).unwrap(), args[2] == "1"),
            "nmeaver" => config_nmea_version(&mut u, u8::from_str_radix(args[1], 16).unwrap()),
            "sat" => config_satellites(&mut u, cfg, warn),
            "tp" => config_time_pulse(&mut u, cfg, warn),
            "nav" => config_nav_engine(&mut u, cfg, warn),
            "timing" => config_for_timing(&mut u, cfg, warn),
            other => panic!("unknown command {other}"),
        };
        (r, u.t.sets.clone())
    }

    #[test]
    fn commands_match_the_real_c_functions() {
        let mut n = 0;
        for line in GOLDEN.lines().filter(|l| !l.is_empty()) {
            let f: Vec<&str> = line.split('\t').collect();
            let (name, args, ini, recv, stateful, c_res, c_warn, c_sent) = (f[0], f[1], f[2].replace("\\n", "\n"), f[3], f[4] == "1", f[5], f[6], f.get(7).copied().unwrap_or(""));
            let bodies: HashMap<(u8, u8), Vec<u8>> = recv.split_whitespace().map(|kv| {
                let (k, v) = kv.split_once('=').unwrap();
                let k = hex_pair(k);
                (k, if v.is_empty() { vec![] } else { hex(v) })
            }).collect();
            let cfg = Config::parse_ini(&ini);
            let mut warn = vec![];
            let args: Vec<&str> = args.split_whitespace().collect();
            let (r, sets) = run_case(&args, &cfg, bodies, stateful, &mut warn);

            assert_eq!(r.is_ok(), c_res == "ok", "{name}: Rust {r:?}, C {c_res}");
            let want: Vec<Vec<u8>> = if c_sent.is_empty() { vec![] } else { c_sent.split('|').map(hex).collect() };
            assert_eq!(sets, want, "{name}: frames sent");
            assert_eq!(warn.len().to_string(), c_warn, "{name}: warnings {warn:?}");
            n += 1;
        }
        assert_eq!(n, 53);
    }

    /// Compact hex ("B562...") to bytes.
    fn hex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    fn hex_pair(k: &str) -> (u8, u8) {
        (u8::from_str_radix(&k[0..2], 16).unwrap(), u8::from_str_radix(&k[2..4], 16).unwrap())
    }

    /// A transport that can be hung up and reopened, to test `with_recovery` without a device.
    struct Flaky { hung: bool, reopens: u32, comes_back: bool }
    impl Transport for Flaky {
        fn now(&self) -> Duration { Duration::ZERO }
        fn pause(&mut self, _d: Duration) {}
        fn read_byte(&mut self, _t: Duration) -> Result<Option<u8>, String> { Ok(None) }
        fn write_all(&mut self, _d: &[u8]) -> Result<(), String> { Ok(()) }
        fn flush_rx(&mut self) -> Result<(), String> { Ok(()) }
        fn set_baud(&mut self, _b: u32) -> Result<(), String> { Ok(()) }
        fn baud(&self) -> u32 { 115200 }
        fn hung_up(&self) -> bool { self.hung }
        fn reopen(&mut self, _w: Duration, _l: Duration) -> Result<(), String> {
            self.reopens += 1;
            if self.comes_back { self.hung = false; Ok(()) } else { Err("serial device did not come back: gone".into()) }
        }
    }

    #[test]
    fn with_recovery_reopens_a_dead_port_and_retries_once() {
        let mut u = Ubx::new(Flaky { hung: true, reopens: 0, comes_back: true });
        let mut runs = 0;
        let r = with_recovery(&mut u, |u| { runs += 1; if u.hung_up() { Err(crate::serial::HUNG_UP_MSG.to_string()) } else { Ok(()) } });
        assert_eq!(r, Ok(()));
        assert_eq!((runs, u.t.reopens), (2, 1));
    }

    #[test]
    fn with_recovery_leaves_other_failures_alone() {
        let mut u = Ubx::new(Flaky { hung: false, reopens: 0, comes_back: true });
        let mut runs = 0;
        let r = with_recovery(&mut u, |_| { runs += 1; Err("received NAK".to_string()) });
        assert_eq!(r, Err("received NAK".to_string()));
        assert_eq!((runs, u.t.reopens), (1, 0), "a usable port is not reopened and the action is not repeated");
    }

    #[test]
    fn with_recovery_reports_a_device_that_does_not_come_back() {
        let mut u = Ubx::new(Flaky { hung: true, reopens: 0, comes_back: false });
        let mut runs = 0;
        let r = with_recovery(&mut u, |_| { runs += 1; Err(crate::serial::HUNG_UP_MSG.to_string()) });
        assert_eq!(r, Err("serial port is not usable and could not be reopened: serial device did not come back: gone".to_string()));
        assert_eq!(runs, 1);
    }

    #[test]
    fn with_recovery_tries_the_action_only_twice() {
        let mut u = Ubx::new(Flaky { hung: true, reopens: 0, comes_back: true });
        let mut runs = 0;
        let r = with_recovery(&mut u, |u| { runs += 1; u.t.hung = true; Err("still dead".to_string()) });
        assert_eq!(r, Err("still dead".to_string()));
        assert_eq!((runs, u.t.reopens), (2, 1));
    }
}
