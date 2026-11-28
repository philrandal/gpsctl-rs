//! The gpsctl command line: the same options, parsed the same way (slOptions), run in the same order, with the same
//! recovery when the serial port goes away.  `run(args)` returns the process exit status.
//!
//! Differences from the C program, on purpose:
//! - extra environment variables: GPSCTL_CONF (config file, default /etc/gpsctl.conf) and GPSCTL_TRACE=1 (print every
//!   byte sent and received to stderr);
//! - a hung-up port is reopened and the action retried, as in C, except for --reset: a cold start can make the device
//!   vanish by itself, and doing it twice would throw away what the receiver had just started to rebuild.

use crate::config::Config;
use crate::serial::{other_users, SttyPort, Traced, Transport, VALID_BAUDS};
use crate::sync::{self, SyncMethod};
use crate::ubx_io::Ubx;
use crate::{cfg_prt, commands, queries};
use std::io::Write;
use std::time::{Duration, Instant};

pub const NAME: &str = "gpsctl";
pub const VERSION: &str = "2.1.0";

type Chain = Vec<String>;

fn one(msg: impl Into<String>) -> Chain { vec![msg.into()] }

/// The library's errors are "wrapper: cause" strings; as a chain, so verbosity 1 can show just the root cause.
fn split_chain(msg: &str) -> Chain {
    // The library words a poll that gets no answer its own way ("CFG-PRT poll: timed out waiting for a UBX message");
    // C's pollUbx() says "Problem receiving poll response" caused by "timed out while waiting to receive UBX message".
    // Only that common case is translated, so the verbose output matches; other library errors keep their own words.
    const TIMEOUT: &str = "timed out while waiting to receive UBX message";
    const POLL: &str = "Problem receiving poll response";
    let mut out: Chain = Vec::new();
    for part in msg.split(": ") {
        // ": " also occurs inside one message ("unexpected CFG-NAV5 message body size: 1 bytes"): not a new link
        if part.ends_with(" bytes") && part.trim_end_matches(" bytes").chars().all(|c| c.is_ascii_digit()) && !out.is_empty() {
            let last = out.pop().unwrap();
            out.push(format!("{last}: {part}"));
            continue;
        }
        match part {
            // (already in C's words, from send_acked)
            "timed out while waiting to receive UBX message" => out.push(part.to_string()),
            "timed out waiting for a UBX message" => {
                // (only poll timeouts reach here, and C always names the poll, once)
                if out.last().map(|l| l.as_str()) != Some(POLL) { out.push(POLL.to_string()); }
                out.push(TIMEOUT.to_string());
            }
            t if t.ends_with(" poll") && t.starts_with("CFG-") => out.push(POLL.to_string()),
            t => out.push(t.to_string()),
        }
    }
    // "CFG-xxx poll" followed by the timeout would now say the poll twice
    out.dedup_by(|b, a| a == POLL && b == POLL);
    out
}

fn wrap(outer: &str, mut cause: Chain) -> Chain {
    let mut c = vec![outer.to_string()];
    c.append(&mut cause);
    c
}

// ---------------------------------------------------------------------------------------------------------------
// option definitions (the same table as getOptionDefs() in gpsctl.c, plus the three that slOptions adds)

#[derive(Clone, Copy, PartialEq, Debug)]
enum Arg { None, Optional, Required }

#[derive(Clone, Copy, PartialEq, Debug)]
enum Id { Version, Usage, Help, Verbose, Quiet, Port, Json, SharedPort, Autobaud, Baud, MinBaud, MaxBaud, Sync, FactoryReset, NewBaud, Nmea, Query, Galileo, Satellites, Timing, Save, Reset, Echo }

struct Def {
    id: Id,
    max: u32,
    long: &'static str,
    short: char, // '\0' for none
    arg: Arg,
    arg_name: &'static str,
    help: &'static str,
}

const fn def(id: Id, max: u32, long: &'static str, short: char, arg: Arg, arg_name: &'static str, help: &'static str) -> Def {
    Def { id, max, long, short, arg, arg_name, help }
}

// (the order matters: parsing, constraint checking and the actions all run in this order)
const DEFS: [Def; 23] = [
    def(Id::Version, 1, "version", 'V', Arg::None, "", "display the version of this program"),
    def(Id::Usage, 1, "usage", 'U', Arg::None, "", "display a short description of this program's usage"),
    def(Id::Help, 1, "help", '?', Arg::None, "", "display this help message"),
    def(Id::Verbose, 2, "verbose", 'v', Arg::None, "", "get verbose messages"),
    def(Id::Quiet, 1, "quiet", 'q', Arg::None, "", "suppress normal output"),
    def(Id::Port, 1, "port", 'p', Arg::Required, "device", "specify the port device (default is '/dev/serial0')"),
    def(Id::Json, 3, "json", 'j', Arg::None, "", "select JSON query output"),
    def(Id::SharedPort, 1, "shared_port", '\0', Arg::None, "", "allow use of a serial port that another process has open"),
    def(Id::Autobaud, 1, "autobaud", 'a', Arg::Optional, "method", "infer baud rate using ascii, nmea, ubx data (default is ubx)"),
    def(Id::Baud, 1, "baud", 'b', Arg::Required, "baud rate", "specify different host baud rate; any standard rate is allowed"),
    def(Id::MinBaud, 1, "minbaud", 'M', Arg::Required, "baud rate", "specify minimum baud rate for autobaud (default is '9600', or \"autobaud minimum\" in gpsctl.conf); any standard rate is allowed"),
    def(Id::MaxBaud, 1, "maxbaud", 'X', Arg::Required, "baud rate", "specify maximum baud rate for autobaud (default is '921600', or \"autobaud maximum\" in gpsctl.conf); any standard rate is allowed"),
    def(Id::Sync, 1, "sync", 's', Arg::Optional, "method", "synchronize using ascii, nmea, ubx data (default is ubx)"),
    def(Id::FactoryReset, 1, "factory_reset", '\0', Arg::Required, "yes", "restore the GPS to its factory defaults, erasing its saved configuration; the GPS goes back to 9600 baud"),
    def(Id::NewBaud, 1, "newbaud", 'B', Arg::Required, "baud rate", "change the U-Blox and host baud rates to the specified standard rate (with -a, that rate is also tried first)"),
    def(Id::Nmea, 1, "nmea", 'n', Arg::Required, "yes/no", "configure whether the GPS sends NMEA data to the serial port (see \"Yes or no values\" below)"),
    def(Id::Query, 1, "query", 'Q', Arg::Required, "query type", "query the GPS for info (see \"Query types\" below)"),
    def(Id::Galileo, 1, "galileo", '\0', Arg::None, "", "configure the GPS for Galileo Satellites"),
    def(Id::Satellites, 1, "satellites", '\0', Arg::None, "", "configure the GPS Satellites only"),
    def(Id::Timing, 1, "configure_for_timing", '\0', Arg::None, "", "configure the GPS for maximum timing precision"),
    def(Id::Save, 1, "save_config", '\0', Arg::None, "", "save the GPS configuration to battery-backed RAM, and to flash, EEPROM and SPI flash if the module has them"),
    def(Id::Reset, 1, "reset", '\0', Arg::None, "", "reset the GPS"),
    def(Id::Echo, 1, "echo", 'e', Arg::Optional, "seconds", "echo human-readable GPS data to stdout for the given time (0 or unspecified means forever)"),
];

const ADDED_INFO: &str = "Additional information:\n\
\n\
\x20 Query types:\n\
\x20   fix         returns position, altitude, and time information\n\
\x20   version     returns hardware and software version of the GPS\n\
\x20   config      returns GPS configuration information\n\
\x20   satellites  returns currently visible satellite information from the GPS\n\
\n\
\x20 Yes or no values:\n\
\x20   yes         indicated by an initial character of 'y', 'Y', 't', 'T', or '1', or by 'on'\n\
\x20   no          indicated by anything else\n\
\n\
Examples:\n\
\x20  gpsctl --port=/dev/serial1 --baud 9600 --echo=5\n\
\x20     Selects the serial port \"/dev/serial1\", sets the host serial port baud rate to 9600, and\n\
\x20     echoes any received characters (presumably NMEA data) to stdout.\n\
\x20  gpsctl -p /dev/serial1 -b 9600 -e=5\n\
\x20     Exactly the same as the preceding example, using short options.\n";

fn option_name(d: &Def) -> String {
    let mut s = String::new();
    if d.short != '\0' {
        s.push('-');
        s.push(d.short);
    }
    if d.short != '\0' && !d.long.is_empty() {
        s.push_str(", ");
    }
    if !d.long.is_empty() {
        s.push_str("--");
        s.push_str(d.long);
    }
    s
}

fn help_text() -> String {
    let mut out = format!("Options for {NAME}:\n\n");
    let mut max_width = 0;
    for d in DEFS.iter() {
        let mut w = 0;
        if d.short != '\0' { w += 2; }
        if !d.long.is_empty() { w += 2 + d.long.len(); }
        if d.short != '\0' && !d.long.is_empty() { w += 2; }
        if d.arg != Arg::None { w += 3 + d.arg_name.len(); }
        max_width = max_width.max(w);
    }
    let (mut got_req, mut got_opt) = (false, false);
    for d in DEFS.iter() {
        let mut name = option_name(d);
        if name.as_bytes().get(1) == Some(&b'-') {
            name = format!("    {name}");
        }
        let namearg = match d.arg {
            Arg::None => name,
            Arg::Required => { got_req = true; format!("{name} <{}>", d.arg_name) }
            Arg::Optional => { got_opt = true; format!("{name}[={}]", d.arg_name) }
        };
        let repeat = if d.max == 1 { String::new() } else { format!(" (may repeat up to {} times)", d.max) };
        out += &format!("  {:<w$} {}{}\n", namearg, d.help, repeat, w = max_width + 2);
    }
    if got_opt || got_req { out.push('\n'); }
    if got_req { out += "  Arguments in <angle brackets> are required; they MUST be supplied.\n"; }
    if got_opt { out += "  Arguments in [square brackets] are optional; they may be omitted.\n"; }
    out.push('\n');
    out += ADDED_INFO;
    out
}

fn usage_text() -> String {
    let (mut short_no, mut short_arg, mut long_no, mut long_arg) = (String::new(), String::new(), String::new(), String::new());
    for d in DEFS.iter() {
        if d.short != '\0' {
            if d.arg == Arg::None {
                if short_no.is_empty() { short_no.push('-'); }
                short_no.push(d.short);
            } else {
                if !short_arg.is_empty() { short_arg.push(' '); }
                short_arg.push('-');
                short_arg.push(d.short);
                if d.arg == Arg::Required { short_arg += &format!(" <{}>", d.arg_name); } else { short_arg += &format!("[={}]", d.arg_name); }
            }
        }
        if !d.long.is_empty() {
            if d.arg == Arg::None {
                if !long_no.is_empty() { long_no.push(' '); }
                long_no += &format!("--{}", d.long);
            } else {
                if !long_arg.is_empty() { long_arg.push(' '); }
                long_arg += &format!("--{}", d.long);
                if d.arg == Arg::Required { long_arg += &format!(" <{}>", d.arg_name); } else { long_arg += &format!("[={}]", d.arg_name); }
            }
        }
    }
    format!("Usage:\n  With short options: {NAME} {short_no} {short_arg}\n  With long options: {NAME} {long_no} {long_arg}\n")
}

// ---------------------------------------------------------------------------------------------------------------
// command-line parsing (parsePhase() in sl_options.c)

struct Record {
    def: usize, // index into DEFS
    arg: Option<String>,
}

fn parse_error(msg: String) -> Chain {
    one(format!("command line options processing error during parsing phase: {msg}."))
}

fn add_record(recs: &mut Vec<Record>, def: usize) -> Result<(), Chain> {
    if recs.len() >= 100 {
        return Err(parse_error("exceeded maximum number of options (100)".to_string()));
    }
    let hits = recs.iter().filter(|r| r.def == def).count() as u32;
    if hits >= DEFS[def].max {
        return Err(parse_error(format!("too many occurrences of option \"{}\": max is {}", option_name(&DEFS[def]), DEFS[def].max)));
    }
    recs.push(Record { def, arg: None });
    Ok(())
}

/// Long options match by prefix (the first definition whose name starts the text wins), so "--port=x" and even
/// "--portx" find "port"; whatever follows the name must then be "=argument".
fn find_long(text: &str) -> Option<usize> {
    DEFS.iter().position(|d| !d.long.is_empty() && text.starts_with(d.long))
}

/// Returns the option records and the first command-line word that option processing did not use (None if there is
/// none).  slOptions moves each unrecognised word to the end of the argument list, but anything after "--" stays where
/// it is, so the words after "--" come before the moved ones.
fn parse_args(args: &[String]) -> Result<(Vec<Record>, Option<String>), Chain> {
    let mut recs: Vec<Record> = Vec::new();
    let mut moved: Vec<String> = Vec::new();
    let mut after_dashes: Vec<String> = Vec::new();
    let mut i = 1;
    while i < args.len() {
        let cur = args[i].as_str();
        let wants_arg = recs.last().map_or(false, |r| DEFS[r.def].arg == Arg::Required && r.arg.is_none());
        if wants_arg {
            recs.last_mut().unwrap().arg = Some(cur.to_string());
        } else if i == 1 && cur == "--?" {
            // (debug tracing in C; nothing to trace here)
        } else if cur == "--" {
            after_dashes = args[i + 1..].to_vec();
            break;
        } else if cur.starts_with('-') {
            if cur.len() > 2 && cur.as_bytes()[1] == b'-' {
                let text = &cur[2..];
                let Some(d) = find_long(text) else {
                    return Err(parse_error(format!("invalid option \"--{text}\" in argument {i}")));
                };
                add_record(&mut recs, d)?;
                let rest = &text[DEFS[d].long.len()..];
                if !rest.is_empty() {
                    if let Some(value) = rest.strip_prefix('=') {
                        if DEFS[d].arg != Arg::None {
                            recs.last_mut().unwrap().arg = Some(value.to_string());
                        } else {
                            return Err(parse_error(format!("unexpected argument (\"{value}\") for long option (\"{}\")", DEFS[d].long)));
                        }
                    } else {
                        return Err(parse_error(format!("invalid option argument (possibly missing the \"=\"): \"{rest}\"")));
                    }
                }
            } else if cur.len() == 1 {
                // a lone "-" would be the option whose short name is a space: there isn't one
                return Err(parse_error(format!("invalid option \"-\" in argument {i}")));
            } else {
                let chars: Vec<char> = cur[1..].chars().collect();
                let mut k = 0;
                while k < chars.len() {
                    let Some(d) = DEFS.iter().position(|d| d.short == chars[k]) else {
                        return Err(parse_error(format!("invalid option \"-{}\" in argument {i}", chars[k])));
                    };
                    add_record(&mut recs, d)?;
                    let more = k + 1 < chars.len();
                    if DEFS[d].arg != Arg::None && more {
                        k += 1;
                        if chars[k] == '=' { k += 1; }
                        recs.last_mut().unwrap().arg = Some(chars[k..].iter().collect());
                        k = chars.len();
                    } else {
                        k += 1;
                    }
                }
            }
        } else {
            // a non-option argument: slOptions moves it to the end, and gpsctl refuses it in the constraint phase
            moved.push(cur.to_string());
        }
        i += 1;
    }
    if let Some(r) = recs.last() {
        if DEFS[r.def].arg == Arg::Required && r.arg.is_none() {
            return Err(parse_error(format!("option [{}] missing required argument", option_name(&DEFS[r.def]))));
        }
    }
    let first_unused = after_dashes.into_iter().chain(moved).next();
    Ok((recs, first_unused))
}

// ---------------------------------------------------------------------------------------------------------------
// the program's state (clientData_slOptions) and the option parsers

#[derive(Clone, Copy, PartialEq, Debug)]
enum Query { Fix, Version, Config, Satellites }

struct ClientData {
    verbosity: i32,
    sync_method: SyncMethod,
    port: Option<String>,
    baud: u32, // 0: keep the port's present rate
    min_baud: u32,
    max_baud: u32,
    new_baud: u32,
    nmea: i32, // -1: leave the receiver's NMEA setting alone
    json: bool,
    echo_seconds: i32,
    query: Query,
    shared_port: bool,
    ubx_synchronized: bool,
    nmea_applied: bool,
}

fn parse_msg(def: usize, msg: String) -> Chain {
    // (parseMsgHelper(), wrapped by the "parsing phase" error)
    wrap("command line options processing error during parsing phase: fatal error.", one(format!("option parsing problem: [{}] {msg}", option_name(&DEFS[def]))))
}

/// argToInt(): the whole argument must be a decimal int.  Err(1): not an integer; Err(2): out of range.
fn arg_to_int(arg: &str) -> Result<i32, u8> {
    if arg.is_empty() { return Err(1); }
    let t = arg.strip_prefix('+').unwrap_or(arg);
    let digits = t.strip_prefix('-').unwrap_or(t);
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) { return Err(1); }
    // strtol() skips leading white space, and accepts one sign; both cases above reject spaces, as C does for the
    // trailing part (the leading part C would accept is not worth reproducing: " 5" is not a sensible argument)
    match t.parse::<i64>() {
        Ok(n) if n >= i32::MIN as i64 && n <= i32::MAX as i64 => Ok(n as i32),
        _ => Err(2),
    }
}

fn parse_baud(def: usize, arg: &str) -> Result<u32, Chain> {
    match arg_to_int(arg) {
        Err(1) => Err(parse_msg(def, format!("the specified baud rate (\"{arg}\") is not an integer"))),
        Err(_) => Err(parse_msg(def, format!("the specified baud rate (\"{arg}\") is out of range"))),
        Ok(b) => {
            if b > 0 && VALID_BAUDS.contains(&(b as u32)) {
                Ok(b as u32)
            } else {
                Err(parse_msg(def, format!("the specified baud rate (\"{arg}\") is not a valid one (4800, 9600, 19200, etc.)")))
            }
        }
    }
}

/// setSyncMethod(): the argument may be any prefix of ascii, nmea or ubx.
fn sync_method_from(arg: &str) -> Option<SyncMethod> {
    if arg.is_empty() { return None; }
    [("ascii", SyncMethod::Ascii), ("nmea", SyncMethod::Nmea), ("ubx", SyncMethod::Ubx)]
        .into_iter()
        .find(|(name, _)| name.starts_with(arg))
        .map(|(_, m)| m)
}

fn parse_bool(arg: &str) -> i32 {
    let b = arg.as_bytes();
    if b.is_empty() { return 0; }
    let mut yes = b"yYtT1".contains(&b[0]);
    if b.len() > 1 && b[0].to_ascii_lowercase() == b'o' && b[1].to_ascii_lowercase() == b'n' { yes = true; }
    yes as i32
}

fn device_is_terminal(path: &str) -> Result<(), String> {
    use std::os::unix::fs::{FileTypeExt, OpenOptionsExt};
    use std::os::unix::io::AsRawFd;
    extern "C" { fn isatty(fd: i32) -> i32; }
    let meta = std::fs::metadata(path).map_err(|e| format!("stat() returned error on \"{path}\": {}", errno_text(&e)))?;
    let ft = meta.file_type();
    if !ft.is_block_device() && !ft.is_char_device() { return Err(format!("\"{path}\" is not a device")); }
    if !ft.is_char_device() { return Err(format!("\"{path}\" is not a character mode device")); }
    let f = std::fs::OpenOptions::new().read(true).write(true).custom_flags(0o4000 | 0o400) // O_NONBLOCK | O_NOCTTY
        .open(path).map_err(|e| format!("can't open \"{path}\": {}", errno_text(&e)))?;
    if unsafe { isatty(f.as_raw_fd()) } != 1 {
        return Err(format!("can't retrieve terminal attributes for \"{path}\": Inappropriate ioctl for device"));
    }
    Ok(())
}

// ---------------------------------------------------------------------------------------------------------------
// the session: the port, the program state and the configuration file

struct Session {
    cd: ClientData,
    cfg: Config,
    u: Option<Ubx<Box<dyn Transport>>>,
}

macro_rules! v1 { ($s:expr) => { $s.cd.verbosity >= 1 }; }
macro_rules! v2 { ($s:expr) => { $s.cd.verbosity >= 2 }; }

fn print_chain(chain: &Chain, include_caused: bool) {
    // printReturn(): the root cause first, as "Error: ...", then each outer message as "   Caused: ..."
    let mut it = chain.iter().rev();
    if let Some(root) = it.next() {
        println!("Error: {root}");
    }
    if include_caused {
        for c in it {
            println!("   Caused: {c}");
        }
    }
}

fn current_baud(path: &str) -> Option<u32> {
    let out = std::process::Command::new("stty").arg("-F").arg(path).arg("speed").output().ok()?;
    if !out.status.success() { return None; }
    String::from_utf8_lossy(&out.stdout).trim().parse().ok()
}

fn open_port(s: &mut Session) -> Result<(), String> {
    let port = s.cd.port.clone().unwrap_or_default();
    if !s.cd.shared_port {
        let others = other_users(&port);
        if !others.is_empty() {
            let list: Vec<String> = others.iter().map(|(p, n)| format!("{n} (pid {p})")).collect();
            return Err(format!("\"{port}\" is already open in another process: {} (use --shared_port to override)", list.join(", ")));
        }
    }
    let baud = if s.cd.baud != 0 {
        s.cd.baud
    } else {
        // keep whatever rate the port is already using (so an autobaud run can still proceed if it is unusual)
        match current_baud(&port) {
            Some(b) if VALID_BAUDS.contains(&b) => b,
            _ => 9600,
        }
    };
    let port_obj = SttyPort::open_shared(&port, baud).map_err(|e| format!("can't open \"{port}\": {e}"))?;
    if v2!(s) { println!("Serial port (\"{port}\") open..."); }
    let t: Box<dyn Transport> = if std::env::var("GPSCTL_TRACE").map_or(false, |v| v == "1") { Box::new(Traced::new(port_obj)) } else { Box::new(port_obj) };
    s.u = Some(Ubx::new(t));
    if v2!(s) { println!("Serial port open and configured..."); }
    Ok(())
}

// ---------------------------------------------------------------------------------------------------------------
// synchronization and the NMEA setting

/// syncSerial()
fn sync_serial(s: &mut Session, method: SyncMethod) -> Result<bool, Chain> {
    let u = s.u.as_mut().unwrap();
    let baud = u.t.baud();
    let max_ms = sync::budget_ms(baud, 1500);
    if method == SyncMethod::Ubx && s.cd.ubx_synchronized {
        return Ok(true);
    }
    let ok = sync::sync(u, method, max_ms, baud).map_err(one)?;
    if method == SyncMethod::Ubx && ok { s.cd.ubx_synchronized = true; }
    Ok(ok)
}

/// applyNMEASetting(): the NMEA on/off setting from the config file or -n, once.
fn apply_nmea_setting(s: &mut Session) -> Result<(), Chain> {
    if s.cd.nmea < 0 || s.cd.nmea > 1 || s.cd.nmea_applied { return Ok(()); }
    let on = s.cd.nmea == 1;
    let verbosity = s.cd.verbosity;
    commands::set_nmea_data_say(s.u.as_mut().unwrap(), on, &mut |level, text| {
        if verbosity >= level as i32 { println!("{text}"); }
    })
        .map_err(|e| wrap(&format!("failed to turn NMEA data {}", if on { "on" } else { "off" }), split_chain(&e)))?;
    s.cd.nmea_applied = true;
    Ok(())
}

/// syncAndApplyNMEA(): UBX synchronization (up to three attempts when it merely times out), then the NMEA setting.
fn sync_and_apply_nmea(s: &mut Session) -> Result<(), Chain> {
    let mut r = sync_serial(s, SyncMethod::Ubx).map_err(|c| wrap("could not synchronize UBX protocol", c))?;
    let mut retry = 0;
    while !r && retry < 2 {
        if v1!(s) { println!("UBX synchronization timed out, retrying ({} of 2)...", retry + 1); }
        r = sync_serial(s, SyncMethod::Ubx).map_err(|c| wrap("could not synchronize UBX protocol", c))?;
        retry += 1;
    }
    if !r { return Err(one("could not synchronize UBX protocol with GPS")); }
    apply_nmea_setting(s)
}

/// applyNMEASettingBestEffort(): the same, but a failure is only reported (at verbosity 1 and above).
fn apply_nmea_best_effort(s: &mut Session) {
    if s.cd.nmea < 0 || s.cd.nmea > 1 { return; }
    if let Err(c) = sync_and_apply_nmea(s) {
        if v1!(s) { print_chain(&c, true); }
    }
}

fn print_warnings(s: &Session, warn: &[String]) {
    if s.cd.verbosity >= 0 {
        for w in warn { eprintln!("Warning: {w}"); }
    }
}

// ---------------------------------------------------------------------------------------------------------------
// the actions

/// What autoBaudRate() says (at -v and above) before it starts trying rates.
fn trying_message(first_baud: u32, max_baud: u32, min_baud: u32) -> String {
    if first_baud == 0 {
        format!("Trying baud rates from {max_baud} to {min_baud}...")
    } else if first_baud <= max_baud && first_baud >= min_baud {
        format!("Trying {first_baud} baud first, then baud rates from {max_baud} to {min_baud}...")
    } else {
        format!("Trying {first_baud} baud first (outside that range), then baud rates from {max_baud} to {min_baud}...")
    }
}

fn action_autobaud(s: &mut Session) -> Result<(), Chain> {
    let method = s.cd.sync_method;
    if v2!(s) { println!("Automatically determining baud rate..."); }
    let (min_baud, max_baud, first_baud) = (s.cd.min_baud, s.cd.max_baud, s.cd.new_baud);
    let u = s.u.as_mut().unwrap();
    // (C closes the port whenever autobaud fails, so the recovery step finds it unusable and reopens it)
    if let Err(e) = u.set_baud(max_baud) {
        s.u = None;
        return Err(wrap("Error when setting terminal options", one(e)));
    }
    let verbosity = s.cd.verbosity;

    // Anything we send at the wrong baud rate is a burst of frame errors in the receiver, and a u-blox that sees more
    // than 100 of them switches its UART receiver off ("UART RX was disabled") for a few minutes, deaf even at the
    // right rate.  So for UBX, first listen at each rate, sending nothing, for the receiver's own NMEA, and poll only
    // at the rate where that is heard.  A receiver with NMEA off says nothing, and then we fall back to polling at
    // every rate...
    let mut found_passively = None;
    if method == SyncMethod::Ubx {
        if verbosity >= 2 { println!("Listening for the receiver's NMEA output first (nothing is sent)..."); }
        // (autoBaudRate() says which rates it will try, each rate as it is tried, and the one that worked)
        if verbosity >= 1 { println!("{}", trying_message(first_baud, max_baud, min_baud)); }
        match sync::auto_baud(s.u.as_mut().unwrap(), min_baud, max_baud, first_baud, SyncMethod::Passive, |b| {
            if verbosity >= 2 { println!("Trying {b} baud..."); }
        }) {
            Ok(b) => {
                if verbosity >= 1 { println!("Synchronized on {b} baud..."); }
                // confirm with a poll, at what should now be the right rate...
                if let Ok(true) = sync::ubx_sync(s.u.as_mut().unwrap(), 4000, b) {
                    found_passively = Some(b);
                } else if verbosity >= 1 {
                    println!("Heard NMEA at {b} baud but the receiver did not answer a UBX poll there; trying every rate...");
                }
            }
            Err(_) => {
                if verbosity >= 2 { println!("Heard no NMEA output at any rate; polling at each rate instead..."); }
            }
        }
        if found_passively.is_none() {
            // the passive pass leaves the port at some rate; start the polling sweep from the top again...
            if let Err(e) = s.u.as_mut().unwrap().set_baud(max_baud) {
                s.u = None;
                return Err(wrap("Error when setting terminal options", one(e)));
            }
        }
    }

    let found = match found_passively {
        Some(b) => Ok(b),
        None => {
            if verbosity >= 1 { println!("{}", trying_message(first_baud, max_baud, min_baud)); }
            let r = sync::auto_baud(s.u.as_mut().unwrap(), min_baud, max_baud, first_baud, method, |b| {
                if verbosity >= 2 { println!("Trying {b} baud..."); }
            });
            if let Ok(b) = r {
                if verbosity >= 1 { println!("Synchronized on {b} baud..."); }
            }
            r
        }
    };
    let baud = match found {
        Ok(b) => b,
        Err(e) => {
            s.u = None;
            return Err(wrap("problem while automatically determining baud rate", one(e)));
        }
    };
    s.cd.baud = baud;

    // cross-check a UBX result: the receiver itself must report the rate we found.  Anything else answering on the
    // port (and re-setting its speed) can make autobaud settle on a rate the receiver isn't using...
    if method == SyncMethod::Ubx {
        let u = s.u.as_mut().unwrap();
        let reported = u.poll(cfg_prt::CLASS, cfg_prt::ID, &[cfg_prt::UART_PORT_ID], 1000).map_err(|e| format!("Problem receiving poll response: {e}"))
            .and_then(|b| cfg_prt::baud(&b).map_err(|e| format!("unexpected CFG-PRT message body size: {} bytes", e.got)));
        match reported {
            Err(e) => {
                if v1!(s) { println!("Warning: could not confirm the detected baud rate with the receiver."); }
                if v2!(s) { print_chain(&split_chain(&e), true); }
            }
            Ok(r) => {
                if r != baud {
                    s.u = None;
                    return Err(one(format!("autobaud found {baud} baud but the receiver reports {r}: something else may be using the port")));
                }
                if v1!(s) { println!("Confirmed: the receiver reports {baud} baud."); }
            }
        }
        // autoBaudRate() has already synchronized using UBX: remember it so the NMEA setup below doesn't do it again
        s.cd.ubx_synchronized = true;
    }
    apply_nmea_best_effort(s);
    Ok(())
}

fn action_baud(s: &mut Session) -> Result<(), Chain> {
    if v2!(s) { println!("Setting baud rate to {}...", s.cd.baud); }
    let baud = s.cd.baud;
    s.u.as_mut().unwrap().set_baud(baud).map_err(|e| wrap("Error when setting terminal options", one(e)))?;
    // let the line go quiet before the first command (waitForQuietLine())
    if let Err(e) = s.u.as_mut().unwrap().t.wait_quiet(crate::serial::QUIET_LINE, crate::serial::QUIET_LINE_MAX) {
        s.u = None;
        return Err(wrap("Error when waiting for the serial line to go quiet", one(e)));
    }
    apply_nmea_best_effort(s);
    Ok(())
}

fn action_new_baud(s: &mut Session) -> Result<(), Chain> {
    if v2!(s) { println!("Changing baud rate to {}...", s.cd.new_baud); }
    let new = s.cd.new_baud;
    let verbosity = s.cd.verbosity;
    cfg_prt::change_baud_say(s.u.as_mut().unwrap(), new, &mut |level, text| {
        if verbosity >= level as i32 { println!("{text}"); }
    }).map_err(|e| wrap("failed to change baud rate", one(e.to_string())))?;
    s.cd.baud = new;
    apply_nmea_best_effort(s);
    Ok(())
}

fn action_sync(s: &mut Session) -> Result<(), Chain> {
    let method = s.cd.sync_method;
    let ok = sync_serial(s, method).map_err(|c| wrap("error while synchronizing on serial data", c))?;
    if ok {
        apply_nmea_best_effort(s);
        Ok(())
    } else {
        Err(one("failed to synchronize"))
    }
}

fn action_nmea(s: &mut Session) -> Result<(), Chain> {
    sync_and_apply_nmea(s)
}

fn action_query(s: &mut Session) -> Result<(), Chain> {
    sync_and_apply_nmea(s)?;
    let json = s.cd.json;
    let u = s.u.as_mut().unwrap();
    let text = match s.cd.query {
        Query::Fix => queries::get_fix(u).map(|f| queries::fix_text(&f, json)).map_err(|c| wrap("Problem obtaining fix from GPS", c)),
        Query::Version => queries::get_version(u).map(|v| { v.warn_if_not_m8(); queries::version_text(&v, json) }).map_err(|c| wrap("Problem obtaining version information from GPS", c)),
        Query::Satellites => queries::get_satellites(u).map(|v| queries::satellites_text(&v, json)).map_err(|c| wrap("Problem obtaining satellites information from GPS", c)),
        Query::Config => queries::get_config(u).map(|c| queries::config_text(&c, json)).map_err(|c| wrap("Problem obtaining configuration information from GPS", c)),
    }
    .map_err(|c| wrap("problem executing GPS query", c))?;
    print!("{text}");
    let _ = std::io::stdout().flush();
    Ok(())
}

fn action_save(s: &mut Session) -> Result<(), Chain> {
    sync_and_apply_nmea(s)?;
    commands::save_config(s.u.as_mut().unwrap())
        .map_err(|e| wrap("problem saving GPS configuration", split_chain(&e)))?;
    if v1!(s) { println!("Saved GPS configuration"); }
    Ok(())
}

fn action_reset(s: &mut Session) -> Result<(), Chain> {
    sync_and_apply_nmea(s)?;
    commands::reset(s.u.as_mut().unwrap()).map_err(|e| wrap("problem resetting the GPS", split_chain(&e)))?;
    // the reset may restore the receiver's default port and NMEA configuration, so anything we previously applied is
    // no longer known to be in effect
    s.cd.ubx_synchronized = false;
    s.cd.nmea_applied = false;
    Ok(())
}

fn action_factory_reset(s: &mut Session) -> Result<(), Chain> {
    sync_and_apply_nmea(s)?;
    let verbosity = s.cd.verbosity;
    let r = commands::factory_reset(s.u.as_mut().unwrap(), &mut |l, t| { if verbosity >= l as i32 { println!("{t}"); } });
    if let Err(e) = r {
        // we don't know what state the link is in, so don't carry on as if it were usable
        s.cd.ubx_synchronized = false;
        s.cd.nmea_applied = false;
        return Err(wrap("problem restoring the GPS to its factory defaults", split_chain(&e)));
    }
    // the port is now at the receiver's default rate, and nothing applied before is in effect any more
    s.cd.baud = commands::FACTORY_BAUD;
    s.cd.ubx_synchronized = true;
    s.cd.nmea_applied = false;
    Ok(())
}

fn action_timing(s: &mut Session) -> Result<(), Chain> {
    sync_and_apply_nmea(s)?;
    let mut warn = Vec::new();
    queries::warn_if_not_m8(s.u.as_mut().unwrap());
    let verbosity = s.cd.verbosity;
    let r = commands::config_for_timing_say(s.u.as_mut().unwrap(), &s.cfg, &mut warn, &mut |l, t| { if verbosity >= l as i32 { println!("{t}"); } });
    print_warnings(s, &warn);
    r.map_err(|e| wrap("problem configuring the GPS for timing", split_chain(&e)))
}

fn action_satellites(s: &mut Session) -> Result<(), Chain> {
    sync_and_apply_nmea(s)?;
    let mut warn = Vec::new();
    queries::warn_if_not_m8(s.u.as_mut().unwrap());
    let verbosity = s.cd.verbosity;
    let r = commands::config_satellites_say(s.u.as_mut().unwrap(), &s.cfg, &mut warn, &mut |l, t| { if verbosity >= l as i32 { println!("{t}"); } });
    print_warnings(s, &warn);
    r.map_err(|e| wrap("problem configuring the GPS satellites", split_chain(&e)))
}

fn action_echo(s: &mut Session) -> Result<(), Chain> {
    if v2!(s) { println!("Starting echo..."); }
    apply_nmea_best_effort(s);
    let start = Instant::now();
    let max = Duration::from_secs(s.cd.echo_seconds.max(0) as u64);
    let forever = s.cd.echo_seconds == 0;
    let out = std::io::stdout();
    while forever || start.elapsed() < max {
        match s.u.as_mut().unwrap().t.read_byte(Duration::from_millis(1000)) {
            Err(e) => return Err(wrap("couldn't read character while echoing", one(e))),
            Ok(None) => continue,
            Ok(Some(b)) => {
                let mut o = out.lock();
                let _ = o.write_all(&[b]);
                if b == b'\n' { let _ = o.flush(); }
            }
        }
    }
    if v2!(s) { println!("\nEnding echo..."); } else { println!(); }
    Ok(())
}

/// actionRecover(): when an action failed because the port is no longer usable, wait for the device to come back,
/// reopen it, and say the action should be tried again.  Ok(false): the port is fine, so the original error stands.
/// reopenSerialPort(): drop the old descriptor (it is dead for good), then keep trying to open the device until it
/// comes back, up to SERIAL_RETURN_MS.  A port that opens but is still hung up does not count (a device that is going
/// away can keep its node long enough to be opened).
fn reopen_session(s: &mut Session) -> Result<(), Chain> {
    let port = s.cd.port.clone().unwrap_or_default();
    s.u = None;
    let deadline = Instant::now() + Duration::from_millis(commands::SERIAL_RETURN_MS);
    loop {
        let failed = match open_port(s) {
            Ok(()) => {
                if !s.u.as_ref().map_or(true, |u| u.hung_up()) { return Ok(()); }
                if v2!(s) { println!("Serial port (\"{port}\") opened but is still hung up..."); }
                s.u = None;
                None
            }
            Err(e) => Some(e),
        };
        if Instant::now() >= deadline {
            return Err(match failed {
                Some(e) => wrap("serial device did not come back", one(e)),
                None => one("serial device is still hung up"),
            });
        }
        std::thread::sleep(Duration::from_millis(commands::SERIAL_LOOK_MS));
    }
}

/// actionRecover(): only a port that is no longer usable (hung up, or closed behind our back by the action: autobaud
/// closes the port when it fails) can be helped by reopening it.  Ok(true) means "reopened: try the action again".
fn recover(s: &mut Session) -> Result<bool, Chain> {
    let port = s.cd.port.clone().unwrap_or_default();
    let hung = s.u.as_ref().map_or(true, |u| u.hung_up());
    if !hung {
        if v2!(s) { println!("Serial port (\"{port}\") is still usable, so not reopening it..."); }
        return Ok(false);
    }
    if v1!(s) { println!("Serial port (\"{port}\") is not usable, reopening it..."); }
    reopen_session(s).map_err(|c| wrap("serial port is not usable and could not be reopened", c))?;
    // whatever we knew about the link is stale: the device may have reset
    s.cd.ubx_synchronized = false;
    s.cd.nmea_applied = false;
    if v1!(s) { println!("Serial port (\"{port}\") reopened, retrying..."); }
    Ok(true)
}

// ---------------------------------------------------------------------------------------------------------------
// the whole run (process_slOptions() and main())

/// actionSetup() reports its problems itself and exits.
fn die(msg: &str) -> ! {
    println!("Error: {msg}");
    let _ = std::io::stdout().flush();
    std::process::exit(1);
}

fn constraint_error(msg: &str) -> Chain {
    wrap("command line options processing error during constraint phase: fatal error.", one(msg))
}

fn load_config() -> Option<Config> {
    let path = std::env::var("GPSCTL_CONF").unwrap_or_else(|_| "/etc/gpsctl.conf".to_string());
    match std::fs::read_to_string(&path) {
        Ok(text) => Some(Config::parse_ini(&text)),
        Err(_) => None,
    }
}

/// Runs gpsctl with the given command line (args[0] is the program name); returns the exit status.
pub fn run(args: &[String]) -> i32 {
    let mut warn = Vec::new();
    let cfg = load_config().unwrap_or_default();
    let verbosity = cfg.int("gpsctl:verbosity", 1, i32::MIN as i64, i32::MAX as i64, &mut warn) as i32;
    let sync_method = match cfg.int("gpsctl:sync method", 3, 1, 3, &mut warn) {
        1 => SyncMethod::Ascii,
        2 => SyncMethod::Nmea,
        _ => SyncMethod::Ubx,
    };
    // getConfigBaud(): a whole number that is one of the standard rates, otherwise the default (with a warning)
    let config_baud = |key: &str, default: u32, warn: &mut Vec<String>| -> u32 {
        let b = cfg.int(key, default as i64, 1, i32::MAX as i64, warn);
        if VALID_BAUDS.contains(&(b as u32)) { b as u32 } else {
            warn.push(format!("{key} = {b} is not a valid baud rate (4800, 9600, 19200, etc.); using the default ({default})"));
            default
        }
    };
    let min_baud = config_baud("gpsctl:autobaud minimum", 9600, &mut warn);
    let max_baud = config_baud("gpsctl:autobaud maximum", 921600, &mut warn);
    // getConfigBool("nmea:enabled", -1): yes, no, or "leave it alone" (not set, or not understood)
    let nmea = match cfg.get("nmea:enabled") {
        None => -1,
        Some(_) => {
            let (mut w1, mut w2) = (Vec::new(), Vec::new());
            let a = cfg.bool("nmea:enabled", false, &mut w1);
            let b = cfg.bool("nmea:enabled", true, &mut w2);
            if a != b { warn.extend(w1); -1 } else { a as i32 }
        }
    };
    for w in &warn { eprintln!("Warning: {w}"); }

    let mut s = Session {
        cd: ClientData {
            verbosity, sync_method, port: None, baud: 0, min_baud, max_baud, new_baud: 0, nmea, json: false, echo_seconds: 0,
            query: Query::Fix, shared_port: false, ubx_synchronized: false, nmea_applied: false,
        },
        cfg,
        u: None,
    };

    let result = process(&mut s, args);
    let status = match result {
        Ok(()) => 0,
        Err(chain) => {
            match s.cd.verbosity {
                0 | 1 => print_chain(&chain, false),
                _ => print_chain(&chain, true),
            }
            1
        }
    };
    let _ = std::io::stdout().flush();
    status
}

fn process(s: &mut Session, args: &[String]) -> Result<(), Chain> {
    let (recs, first_unused) = parse_args(args)?;
    let present = |id: Id| recs.iter().any(|r| DEFS[r.def].id == id);

    // help, version and usage only inform: run them, and nothing else
    let mut informational = false;
    for (di, d) in DEFS.iter().enumerate() {
        for r in recs.iter().filter(|r| r.def == di) {
            let _ = r;
            match d.id {
                Id::Help => { print!("{}", help_text()); informational = true; }
                Id::Version => { println!("{NAME} version {VERSION}"); informational = true; }
                Id::Usage => { print!("{}", usage_text()); informational = true; }
                _ => {}
            }
        }
    }
    if informational { return Ok(()); }

    // parsing, in definition order
    for (di, d) in DEFS.iter().enumerate() {
        for r in recs.iter().filter(|r| r.def == di) {
            let arg = r.arg.as_deref();
            match d.id {
                Id::Verbose => s.cd.verbosity += 1,
                Id::Port => {
                    let a = arg.unwrap_or("");
                    s.cd.port = Some(a.to_string());
                    if let Err(e) = device_is_terminal(a) {
                        return Err(wrap("command line options processing error during parsing phase: fatal error.",
                            wrap(&format!("device \"{a}\" is not a terminal"), one(e))));
                    }
                }
                Id::Json => s.cd.json = true,
                Id::SharedPort => s.cd.shared_port = true,
                Id::Autobaud | Id::Sync => {
                    if let Some(a) = arg {
                        match sync_method_from(a) {
                            Some(m) => s.cd.sync_method = m,
                            None => return Err(parse_msg(di, format!(
                                "the specified baud rate inference method (\"{a}\") is unrecognizable; should be \"ascii\", \"nmea\", or \"ubx\""))),
                        }
                    }
                }
                Id::Baud => s.cd.baud = parse_baud(di, arg.unwrap_or(""))?,
                Id::MinBaud => s.cd.min_baud = parse_baud(di, arg.unwrap_or(""))?,
                Id::MaxBaud => s.cd.max_baud = parse_baud(di, arg.unwrap_or(""))?,
                Id::NewBaud => s.cd.new_baud = parse_baud(di, arg.unwrap_or(""))?,
                Id::FactoryReset => {
                    if arg != Some("yes") {
                        return Err(parse_msg(di, "the argument must be the word \"yes\", to confirm that the GPS's saved configuration is to be erased".to_string()));
                    }
                }
                Id::Nmea => s.cd.nmea = parse_bool(arg.unwrap_or("")),
                Id::Query => {
                    let a = arg.unwrap_or("");
                    s.cd.query = match a {
                        "fix" => Query::Fix,
                        "version" => Query::Version,
                        "satellites" => Query::Satellites,
                        "config" => Query::Config,
                        _ => return Err(parse_msg(di, format!("the specified query type (\"{a}\") is not a valid query type"))),
                    };
                }
                Id::Echo => {
                    s.cd.echo_seconds = match arg {
                        None => 0,
                        Some(a) => match arg_to_int(a) {
                            Ok(n) => n,
                            Err(1) => return Err(parse_msg(di, format!("the specified seconds (\"{a}\") is not an integer"))),
                            Err(_) => return Err(parse_msg(di, format!("the specified seconds (\"{a}\") is out of range"))),
                        },
                    };
                }
                _ => {}
            }
        }
    }

    // constraints: gpsctl takes no positional arguments, so a word left over means the command line was not understood
    // ("-e 5" is one: the optional argument must be attached, "-e5" or "--echo=5")
    if let Some(extra) = first_unused {
        return Err(constraint_error(&format!("unexpected command-line argument \"{extra}\"")));
    }
    for (di, d) in DEFS.iter().enumerate() {
        if !recs.iter().any(|r| r.def == di) { continue; }
        let bad = match d.id {
            Id::Sync if present(Id::Autobaud) => Some("-s, --sync may only be specified if -a, --autobaud is NOT specified"),
            Id::Json if !present(Id::Query) => Some("-j, --json may only be specified if -Q, --query is also specified"),
            Id::Baud if present(Id::Autobaud) => Some("-b, --baud may only be specified if -a, --autobaud is NOT specified"),
            Id::MinBaud if !present(Id::Autobaud) => Some("-M, --minbaud may only be specified if -a, --autobaud is also specified"),
            Id::MaxBaud if !present(Id::Autobaud) => Some("-X, --maxbaud may only be specified if -a, --autobaud is also specified"),
            Id::Echo if s.cd.echo_seconds < 0 => Some("-e, --echo may not be given a negative number of seconds"),
            Id::Verbose if present(Id::Quiet) => Some("-v, --verbosity d may only be specified if -q, --quiet is NOT specified"),
            _ => None,
        };
        if let Some(m) = bad { return Err(constraint_error(m)); }
    }
    // the autobaud range must not be empty; either end may come from gpsctl.conf or the command line, so say which...
    if present(Id::Autobaud) && s.cd.min_baud > s.cd.max_baud {
        return Err(constraint_error(&format!(
            "the minimum autobaud rate ({}, from {}) is above the maximum ({}, from {})",
            s.cd.min_baud, if present(Id::MinBaud) { "-M, --minbaud" } else { "gpsctl.conf or the default" },
            s.cd.max_baud, if present(Id::MaxBaud) { "-X, --maxbaud" } else { "gpsctl.conf or the default" })));
    }

    // setup: find the port and open it
    if s.cd.port.is_none() {
        let p = s.cfg.get("gpsctl:port").unwrap_or("/dev/serial0").to_string();
        if device_is_terminal(&p).is_err() {
            die(&format!("device \"{p}\" is not a terminal"));
        }
        s.cd.port = Some(p);
    }
    if let Err(e) = open_port(s) {
        die(&e);
    }

    // the actions, in definition order, each with one retry after the port has been reopened
    for (di, d) in DEFS.iter().enumerate() {
        for _ in recs.iter().filter(|r| r.def == di) {
            let run_action = |s: &mut Session| -> Result<(), Chain> {
                match d.id {
                    Id::Quiet => { s.cd.verbosity = 0; Ok(()) }
                    Id::Autobaud => action_autobaud(s),
                    Id::Baud => action_baud(s),
                    Id::Sync => action_sync(s),
                    Id::FactoryReset => action_factory_reset(s),
                    Id::NewBaud => action_new_baud(s),
                    Id::Nmea => action_nmea(s),
                    Id::Query => action_query(s),
                    Id::Galileo | Id::Satellites => action_satellites(s),
                    Id::Timing => action_timing(s),
                    Id::Save => action_save(s),
                    Id::Reset => action_reset(s),
                    Id::Echo => action_echo(s),
                    _ => Ok(()),
                }
            };
            let mut r = run_action(s);
            if r.is_err() && d.id != Id::Reset {
                match recover(s) {
                    Ok(true) => r = run_action(s),
                    Ok(false) => {}
                    Err(c) => r = Err(c),
                }
            }
            if let Err(c) = r {
                return Err(wrap("command line options processing error during action phase: fatal error.", c));
            }
        }
    }
    s.u = None; // (closes the port)
    Ok(())
}

/// strerror() text: Rust appends " (os error N)", C does not.
fn errno_text(e: &std::io::Error) -> String {
    let t = e.to_string();
    t.split(" (os error").next().unwrap_or(&t).to_string()
}
