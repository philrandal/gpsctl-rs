//! Poll (and optionally change) a u-blox receiver's NMEA version over a serial port.  std-only; the port is configured
//! by running `stty`, and opened ONCE for both reading and writing (a serial port discards incoming bytes while nothing
//! has it open, so a separate write-then-read can lose the reply).
//!
//!   cargo run --example nmea_tool -- poll
//!   cargo run --example nmea_tool -- set 4.0          # dry run: shows what would be sent
//!   cargo run --example nmea_tool -- set 4.0 --yes    # really sends it
//!   options: --port /dev/ttyAMA0  --baud 115200
//!
//! Stop gpsd / ntpd / gpsctl first: anything else reading the port will steal the reply.
use gpsctl_rs::{cfg_gnss, cfg_msg, cfg_prt, cfg_rate, commands, mon_ver, nav_pvt, nav_sat};
use gpsctl_rs::config::Config;
use gpsctl_rs::serial::{SttyPort, Traced, Transport};
use gpsctl_rs::sync::{auto_baud, SyncMethod};
use gpsctl_rs::ubx_io::{Ubx, UbxError};
use gpsctl_rs::cfg_nmea::{extract_frame, parse_frame, poll_frame, set_version_frame, version, version_from_number, CLASS, ID};
use std::fs::{File, OpenOptions};
use std::io::{ErrorKind, Read, Write};
use std::process::{exit, Command};
use std::time::{Duration, Instant};

const REPLY_TIMEOUT: Duration = Duration::from_secs(3);

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02X}")).collect::<Vec<_>>().join(" ")
}

struct Port {
    file: File,
    buf: Vec<u8>,
}

impl Port {
    fn open(path: &str, baud: u32, force: bool) -> Result<Port, String> {
        // same rule as for the Ubx-based commands: another program on the port steals replies
        let others = gpsctl_rs::serial::other_users(path);
        if !others.is_empty() {
            if !force {
                let list: Vec<String> = others.iter().map(|(p, n)| format!("{n} (pid {p})")).collect();
                return Err(format!("{path} is already open in: {}. Stop it first (e.g. `sudo systemctl stop gpsd.socket gpsd`), or pass --force.", list.join(", ")));
            }
            eprintln!("warning: {path} is also open in {:?}; replies may be stolen", others);
        }
        // min 0 time 1: a read returns after 0.1 s even with no data, so the deadline loop below can run.
        let st = Command::new("stty")
            .args(["-F", path, &baud.to_string(), "raw", "-echo", "-hupcl", "clocal", "min", "0", "time", "1"])
            .status()
            .map_err(|e| format!("can't run stty: {e}"))?;
        if !st.success() {
            return Err(format!("stty failed on {path} (wrong port, or no permission: try sudo / the dialout group)"));
        }
        let file = OpenOptions::new().read(true).write(true).open(path).map_err(|e| format!("can't open {path}: {e}"))?;
        Ok(Port { file, buf: Vec::new() })
    }

    fn send(&mut self, frame: &[u8]) -> Result<(), String> {
        self.file.write_all(frame).and_then(|_| self.file.flush()).map_err(|e| format!("write failed: {e}"))
    }

    /// Read until a valid frame accepted by `want(class, id, body)` arrives, or the timeout passes.
    fn wait_for(&mut self, what: &str, want: impl Fn(u8, u8, &[u8]) -> bool) -> Result<Vec<u8>, String> {
        let deadline = Instant::now() + REPLY_TIMEOUT;
        let mut tmp = [0u8; 512];
        loop {
            while let Some(f) = extract_frame(&mut self.buf) {
                if let Ok((c, i, b)) = parse_frame(&f) {
                    if want(c, i, b) {
                        return Ok(f);
                    }
                }
            }
            if Instant::now() >= deadline {
                return Err(format!("timed out waiting for {what} (is another program using the port? right baud rate?)"));
            }
            match self.file.read(&mut tmp) {
                Ok(n) => self.buf.extend_from_slice(&tmp[..n]),
                Err(e) if e.kind() == ErrorKind::Interrupted => {}
                Err(e) => return Err(format!("read failed: {e}")),
            }
        }
    }

    /// Poll MON-VER and return its body.
    fn poll_version(&mut self) -> Result<Vec<u8>, String> {
        self.send(&mon_ver::poll_frame())?;
        let f = self.wait_for("the MON-VER reply", |c, i, _| c == mon_ver::CLASS && i == mon_ver::ID)?;
        Ok(parse_frame(&f).unwrap().2.to_vec())
    }

    /// Poll CFG-GNSS and return its body.
    fn poll_gnss(&mut self) -> Result<Vec<u8>, String> {
        self.send(&cfg_gnss::poll_frame())?;
        let f = self.wait_for("the CFG-GNSS reply", |c, i, b| c == cfg_gnss::CLASS && i == cfg_gnss::ID && !b.is_empty())?;
        Ok(parse_frame(&f).unwrap().2.to_vec())
    }

    /// Poll any packet: send `poll` and return the body of the first non-empty reply of that class/id.
    fn poll_any(&mut self, poll: &[u8], class: u8, id: u8, what: &str) -> Result<Vec<u8>, String> {
        self.send(poll)?;
        let f = self.wait_for(what, |c, i, b| c == class && i == id && !b.is_empty())?;
        Ok(parse_frame(&f).unwrap().2.to_vec())
    }

    /// Poll CFG-NMEA and return its body.
    fn poll_body(&mut self) -> Result<Vec<u8>, String> {
        self.send(&poll_frame())?;
        let f = self.wait_for("the CFG-NMEA reply", |c, i, b| c == CLASS && i == ID && !b.is_empty())?;
        Ok(parse_frame(&f).unwrap().2.to_vec())
    }
}

fn describe(body: &[u8]) -> Result<(), String> {
    let v = version(body)?;
    println!("  body ({} bytes): {}", body.len(), hex(body));
    println!("  NMEA version byte: 0x{v:02X}  (NMEA {}.{})", v >> 4, v & 0x0F);
    Ok(())
}

/// The serial port for the Ubx-based commands: refuses a port another process (gpsd...) has open unless --force;
/// with --trace, everything sent and received is printed to stderr.
fn open_port(path: &str, baud: u32, force: bool, trace: bool) -> Result<Box<dyn Transport>, String> {
    let port = if force {
        let others = gpsctl_rs::serial::other_users(path);
        if !others.is_empty() {
            eprintln!("warning: {path} is also open in {:?}; replies may be stolen or faked and the baud rate fought over", others);
        }
        SttyPort::open_shared(path, baud).map_err(|e| format!("can't open {path}: {e}"))?
    } else {
        SttyPort::open(path, baud)?
    };
    Ok(if trace { Box::new(Traced::new(port)) } else { Box::new(port) })
}

/// Poll, trying again a few times when no valid reply arrives (only for reads, which are harmless to repeat).
fn poll_retry<T: Transport>(u: &mut Ubx<T>, class: u8, id: u8, body: &[u8], ms: u64, tries: u32) -> Result<Vec<u8>, UbxError> {
    let mut last = Err(UbxError::Timeout);
    for n in 1..=tries {
        last = u.poll(class, id, body, ms);
        match &last {
            Err(UbxError::Timeout) if n < tries => eprintln!("no valid reply (attempt {n} of {tries}), trying again..."),
            _ => break,
        }
    }
    last
}

fn usage() {
    println!("usage: nmea_tool [--port /dev/ttyAMA0] [--baud 115200] poll");
    println!("       nmea_tool [--port ...] [--baud ...] gnss         (CFG-GNSS: which constellations are on, channels)");
    println!("       nmea_tool [--port ...] [--baud ...] ver          (MON-VER: software/hardware/extensions)");
    println!("       nmea_tool [--port ...] [--baud ...] msgs         (CFG-MSG: which NMEA sentences are on for UART1)");
    println!("       nmea_tool [--port ...] [--baud ...] prt          (CFG-PRT: UART baud rate and whether NMEA output is on)");
    println!("       nmea_tool [--port ...] [--baud ...] rate         (CFG-RATE: measurement/navigation rate, time reference)");
    println!("       nmea_tool [--port ...] [--baud ...] fix          (NAV-PVT: time, position, accuracy, corrected like gpsctl)");
    println!("       nmea_tool [--port ...] [--baud ...] sats         (NAV-SAT: satellites, signal strength, used or not)");
    println!("       nmea_tool [--port ...] autobaud [ascii|nmea|ubx] [--min 9600]   (find the receiver's baud rate; changes nothing on it)");
    println!("       nmea_tool [--port ...] [--baud ...] baud <new rate> [--yes]    (change the receiver's UART rate and verify; dry run without --yes)");
    println!("       nmea_tool [--port ...] [--baud ...] set <version> [--yes]");
    println!("       nmea_tool [--port ...] [--baud ...] [--config FILE] <save|reset|sat|tp|nav|timing|nmeadata on|off|msg <0-8> on|off|nmeaver <41>> --yes");
    println!("       (these CHANGE the receiver, so --yes is required; sat/tp/nav/timing read settings from --config, default /etc/gpsctl.conf if present;");
    println!("        'save' (--mask 0xNNNN: only those sections; ioPort 0x01 msgConf 0x02 infMsg 0x04 navConf 0x08 rxmConf 0x10 antConf 0x400) writes the configuration to the receiver's battery-backed RAM, nothing else does; msg ids: GGA 0 GLL 1 GSA 2 GSV 3 RMC 4 VTG 5 GRS 6 GST 7 ZDA 8)");
    println!("  <version> as in gpsctl.conf: 4.1, 41, 4.0, 2.3 ... (gpsctl forces >= 4.1 when BeiDou/Galileo are enabled,");
    println!("  so setting 4.0 here on such a receiver will be changed back the next time gpsctl runs)");
    println!("  without --yes, 'set' only shows what it would send");
    println!("  --trace (autobaud and baud) prints every byte sent and received to stderr");
    println!("  autobaud and baud refuse to run while another program (gpsd, ntpd...) has the port open; --force overrides that");
}

fn run() -> Result<(), String> {
    let mut port = "/dev/ttyAMA0".to_string();
    let mut baud = 115200u32;
    let mut yes = false;
    let mut force = false;
    let mut trace = false;
    let mut min_baud = 9600u32;
    let mut config_path: Option<String> = None;
    let mut save_mask: Option<u32> = None;
    let mut cmd: Vec<String> = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--port" => port = args.next().ok_or("--port needs a value")?,
            "--baud" => baud = args.next().ok_or("--baud needs a value")?.parse().map_err(|_| "bad --baud value")?,
            "--yes" => yes = true,
            "--force" => force = true,
            "--trace" => trace = true,
            "--mask" => {
                let v = args.next().ok_or("--mask needs a value")?;
                let m = match v.strip_prefix("0x").or_else(|| v.strip_prefix("0X")) {
                    Some(h) => u32::from_str_radix(h, 16),
                    None => v.parse(),
                }
                .map_err(|_| "bad --mask value (use e.g. 0x0008)")?;
                if m == 0 || m > 0xFFFF { return Err("--mask must be between 0x0001 and 0xFFFF".into()); }
                save_mask = Some(m);
            }
            "--config" => config_path = Some(args.next().ok_or("--config needs a value")?),
            "--min" => min_baud = args.next().ok_or("--min needs a value")?.parse().map_err(|_| "bad --min value")?,
            "-h" | "--help" => return Ok(usage()),
            _ => cmd.push(a),
        }
    }
    match cmd.first().map(String::as_str) {
        Some("poll") => {
            let mut p = Port::open(&port, baud, force)?;
            println!("CFG-NMEA from {port} at {baud}:");
            describe(&p.poll_body()?)
        }
        Some("gnss") => {
            let mut p = Port::open(&port, baud, force)?;
            let body = p.poll_gnss()?;
            println!("CFG-GNSS from {port} at {baud} ({} byte body):", body.len());
            println!("  header: {}", hex(&body[..body.len().min(4)]));
            for b in cfg_gnss::blocks(&body)? {
                println!("  {:8} {:3}  min {:2}  max {:2}  flags 0x{:08X}", b.name(), if b.enabled() { "on" } else { "off" }, b.min_ch, b.max_ch, b.flags);
            }
            Ok(())
        }
        Some(c @ ("save" | "reset" | "sat" | "tp" | "nav" | "timing" | "nmeadata" | "msg" | "nmeaver")) => {
            let c = c.to_string();
            if !yes {
                return Err(format!("'{c}' changes the receiver's settings; add --yes to do it"));
            }
            let cfg = match config_path.as_deref().or_else(|| std::path::Path::new("/etc/gpsctl.conf").exists().then_some("/etc/gpsctl.conf")) {
                Some(path) => { println!("Using settings from {path}"); Config::parse_ini(&std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?) }
                None => { println!("No config file: using gpsctl's built-in defaults"); Config::new() }
            };
            let onoff = |i: usize| -> Result<bool, String> {
                match cmd.get(i).map(String::as_str) { Some("on") | Some("1") => Ok(true), Some("off") | Some("0") => Ok(false), _ => Err("expected on or off".to_string()) }
            };
            let mut u = Ubx::new(open_port(&port, baud, force, trace)?);
            let mut warn = Vec::new();
            // A port that goes away mid-command (USB adapter unplugged or reset) is reopened and the command tried
            // again, as gpsctl does, except `reset`: a cold start can itself make the device vanish, and doing it
            // twice would throw away what the receiver had just started to rebuild.
            let run_cmd = |u: &mut Ubx<Box<dyn Transport>>, warn: &mut Vec<String>| -> Result<(), String> {
                warn.clear();
                match c.as_str() {
                    "save" => match save_mask {
                        None => commands::save_config(u),
                        Some(m) => {
                            println!("saving only sections 0x{m:04X} (gpsctl itself saves 0x041B)");
                            commands::save_config_mask(u, m)
                        }
                    },
                    "reset" => commands::reset(u),
                    "sat" => commands::config_satellites(u, &cfg, warn),
                    "tp" => commands::config_time_pulse(u, &cfg, warn),
                    "nav" => commands::config_nav_engine(u, &cfg, warn),
                    "timing" => commands::config_for_timing(u, &cfg, warn),
                    "nmeadata" => commands::set_nmea_data(u, onoff(1)?),
                    "msg" => {
                        let id: u8 = cmd.get(1).and_then(|v| v.parse().ok()).ok_or("msg needs an id 0-8")?;
                        commands::enable_nmea_msg(u, cfg_msg::Nmea::from_id(id).ok_or("msg id must be 0-8")?, onoff(2)?)
                    }
                    _ => {
                        let v: f64 = cmd.get(1).and_then(|v| v.parse().ok()).ok_or("nmeaver needs a version, e.g. 4.1")?;
                        let byte = gpsctl_rs::cfg_nmea::version_from_number(v).ok_or("nmeaver must be 2.1, 2.3, 4.0, 4.1 or 4.11")?;
                        commands::config_nmea_version(u, byte)
                    }
                }
            };
            let r = if c == "reset" { run_cmd(&mut u, &mut warn) } else { commands::with_recovery(&mut u, |u| run_cmd(u, &mut warn)) };
            for w in &warn { println!("Warning: {w}"); }
            r?;
            println!("OK: {c} done.");
            Ok(())
        }
        Some("autobaud") => {
            let method = match cmd.get(1).map(String::as_str) { None | Some("ubx") => SyncMethod::Ubx, Some("nmea") => SyncMethod::Nmea, Some("ascii") => SyncMethod::Ascii,
                Some(o) => return Err(format!("unknown sync method {o} (ascii, nmea or ubx)")) };
            let mut u = Ubx::new(open_port(&port, 921600, force, trace)?);
            println!("Trying {:?} synchronization on {port} from 921600 down to {min_baud}...", method);
            let found = auto_baud(&mut u, min_baud, method, |b| println!("  trying {b}"))?;
            println!("Synchronized at {found} baud ({:?} said so).", method);
            // double-check with a different question: what rate does the receiver itself say it is using?
            match poll_retry(&mut u, cfg_prt::CLASS, cfg_prt::ID, &[cfg_prt::UART_PORT_ID], 1000, 3) {
                Ok(body) => match cfg_prt::baud(&body) {
                    Ok(b) if b == found => println!("Confirmed: CFG-PRT says the receiver UART is at {b} baud."),
                    Ok(b) => println!("WARNING: CFG-PRT says the receiver UART is at {b} baud, not {found}. The synchronization was a false positive."),
                    Err(e) => println!("CFG-PRT reply unusable: {e:?}"),
                },
                Err(e) => println!("WARNING: no CFG-PRT reply at {found} baud ({e}). The synchronization may have been a false positive."),
            }
            Ok(())
        }
        Some("baud") => {
            let want: u32 = cmd.get(1).ok_or("baud needs a rate, e.g. 115200")?.parse().map_err(|_| "bad baud rate")?;
            if !gpsctl_rs::serial::VALID_BAUDS.contains(&want) {
                return Err(format!("{want} is not a valid baud rate"));
            }
            let mut u = Ubx::new(open_port(&port, baud, force, trace)?);
            let body = poll_retry(&mut u, cfg_prt::CLASS, cfg_prt::ID, &[cfg_prt::UART_PORT_ID], 1000, 3).map_err(|e| format!("could not read CFG-PRT: {e}"))?;
            let now = cfg_prt::baud(&body).map_err(|e| format!("{e:?}"))?;
            println!("Receiver UART is at {now} baud (host at {baud}).");
            if !yes {
                let f = cfg_prt::set_baud_frame(&body, want).map_err(|e| format!("{e:?}"))?;
                println!("Would send ({} bytes): {}", f.len(), hex(&f));
                println!("then move the host to {want}, re-poll to confirm, and retry once / move the host back if not confirmed.");
                println!("Dry run. Add --yes to do it.  The receiver keeps the new rate until reset/power-off unless saved (this tool never saves);");
                println!("if it ever gets lost: nmea_tool autobaud");
                return Ok(());
            }
            println!("Changing to {want}...");
            cfg_prt::change_baud(&mut u, want).map_err(|e| format!("{e}"))?;
            println!("OK: the receiver confirmed {want} baud.  To undo: nmea_tool --port {port} --baud {want} baud {now} --yes   (or power-cycle)");
            Ok(())
        }
        Some("msgs") => {
            let mut p = Port::open(&port, baud, force)?;
            println!("CFG-MSG (NMEA sentences on UART1) from {port} at {baud}:");
            for m in cfg_msg::Nmea::ALL {
                let body = p.poll_any(&cfg_msg::poll_frame(m), cfg_msg::CLASS, cfg_msg::ID, "a CFG-MSG reply")?;
                let on = cfg_msg::is_enabled(&body).map_err(|e| e.to_string())?;
                println!("  {:?}: {}   (rates I2C/UART1/UART2/USB/SPI: {})", m, if on { "on" } else { "off" }, hex(&body[2..body.len().min(8)]));
            }
            Ok(())
        }
        Some("prt") => {
            let mut p = Port::open(&port, baud, force)?;
            let body = p.poll_any(&cfg_prt::poll_frame(), cfg_prt::CLASS, cfg_prt::ID, "the CFG-PRT reply")?;
            println!("CFG-PRT (UART1) from {port} at {baud} ({} byte body): {}", body.len(), hex(&body));
            println!("  baud rate: {}", cfg_prt::baud(&body).map_err(|e| format!("{e:?}"))?);
            println!("  NMEA output: {}", if cfg_prt::nmea_out(&body).map_err(|e| format!("{e:?}"))? { "on" } else { "off" });
            Ok(())
        }
        Some("rate") => {
            let mut p = Port::open(&port, baud, force)?;
            let body = p.poll_any(&cfg_rate::poll_frame(), cfg_rate::CLASS, cfg_rate::ID, "the CFG-RATE reply")?;
            let r = cfg_rate::decode(&body).ok_or("CFG-RATE body too short")?;
            println!("CFG-RATE from {port} at {baud}: measurement every {} ms, one solution per {} measurement(s), time reference {} ({})",
                r.meas_rate_ms, r.nav_rate, r.time_ref, cfg_rate::time_ref_name(r.time_ref));
            Ok(())
        }
        Some("fix") => {
            let mut p = Port::open(&port, baud, force)?;
            let body = p.poll_any(&nav_pvt::poll_frame(), nav_pvt::CLASS, nav_pvt::ID, "the NAV-PVT reply")?;
            let f = nav_pvt::decode(&body).map_err(|e| format!("unexpected NAV-PVT body size: {} bytes", e.0))?;
            println!("NAV-PVT from {port} at {baud}:");
            println!("  time: {:04}-{:02}-{:02} {:02}:{:02}:{:09.6}  (date valid {}, time valid {}, resolved {}, accuracy {} ns)",
                f.year, f.month, f.day, f.hour, f.minute, (f.second * 1e6).floor() / 1e6, f.date_valid, f.time_valid, f.time_resolved, f.time_accuracy_ns);
            println!("  fix valid: {}  3D: {}  satellites used: {}", f.fix_valid, f.fix_is_3d, f.number_of_satellites_used);
            println!("  position: {:.7}, {:.7}  height {} mm (sea level {} mm)  accuracy h {} mm / v {} mm",
                f.latitude_deg, f.longitude_deg, f.height_above_ellipsoid_mm, f.height_above_sea_level_mm, f.horizontal_accuracy_mm, f.height_accuracy_mm);
            println!("  speed: {} mm/s (+/- {})  heading {:.2} deg (+/- {:.2})", f.ground_speed_mm_s, f.ground_speed_accuracy_mm_s, f.heading_deg, f.heading_accuracy_deg);
            Ok(())
        }
        Some("sats") => {
            let mut p = Port::open(&port, baud, force)?;
            let body = p.poll_any(&nav_sat::poll_frame(), nav_sat::CLASS, nav_sat::ID, "the NAV-SAT reply")?;
            let sats = nav_sat::decode(&body).map_err(|e| format!("unexpected NAV-SAT body size: {} bytes", e.0))?;
            println!("NAV-SAT from {port} at {baud}: {} satellite(s)", sats.len());
            for s in &sats {
                println!("  {:8} {:3}  C/N0 {:2}  elev {:3}  az {:3}  {}  quality {}  health {}", nav_sat::gnss_name(s.gnss_id), s.satellite_id,
                    s.cno, s.elevation, s.azimuth, if s.used { "USED" } else { "    " }, s.signal_quality, s.health);
            }
            Ok(())
        }
        Some("ver") => {
            let mut p = Port::open(&port, baud, force)?;
            let body = p.poll_version()?;
            let v = mon_ver::decode(&body);
            println!("MON-VER from {port} at {baud} ({} byte body):", body.len());
            println!("  software: {}", v.software);
            println!("  hardware: {}", v.hardware);
            for e in &v.extensions {
                println!("  extension: {e}");
            }
            Ok(())
        }
        Some("set") => {
            let value: f64 = cmd.get(1).ok_or("set needs a version, e.g. 4.0")?.parse().map_err(|_| "bad version")?;
            let want = version_from_number(value).ok_or("the version must be 2.1, 2.3, 4.0, 4.1 or 4.11")?;
            let mut p = Port::open(&port, baud, force)?;
            println!("Current CFG-NMEA from {port} at {baud}:");
            let body = p.poll_body()?;
            describe(&body)?;
            let Some(frame) = set_version_frame(&body, want)? else {
                println!("Already 0x{want:02X}; nothing to send.");
                return Ok(());
            };
            let old = version(&body)?;
            if !yes {
                println!("Would send ({} bytes): {}", frame.len(), hex(&frame));
                println!("Dry run. Add --yes to send it.");
                return Ok(());
            }
            println!("Sending ({} bytes): {}", frame.len(), hex(&frame));
            p.send(&frame)?;
            // the receiver answers ACK-ACK (05 01) or ACK-NAK (05 00) naming the message it acknowledges
            let ack = p.wait_for("an ACK", |c, i, b| c == 0x05 && i <= 1 && b == [CLASS, ID])?;
            if parse_frame(&ack).unwrap().1 == 0 {
                return Err("receiver rejected the message (ACK-NAK); nothing changed".into());
            }
            println!("ACK-ACK received. Verifying with a new poll:");
            let after = p.poll_body()?;
            describe(&after)?;
            if version(&after)? == want {
                println!("OK: version is now 0x{want:02X}. To undo: nmea_tool set {}.{} --yes", old >> 4, old & 0x0F);
                Ok(())
            } else {
                Err("receiver ACKed but still reports a different version".into())
            }
        }
        _ => Ok(usage()),
    }
}

fn main() {
    if let Err(e) = run() {
        eprintln!("error: {e}");
        exit(1);
    }
}
