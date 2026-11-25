//! The byte transport under everything else, plus the real serial port.  std-only: `stty` sets the line parameters,
//! and two libc calls std already links (`tcflush`, `tcdrain`) do what stty can't.  Everything above this (UBX framing,
//! polling, synchronizers, autobaud) is generic over `Transport`, so it runs against a simulated receiver in tests.

use std::collections::VecDeque;
use std::fs::{File, OpenOptions};
use std::io::{ErrorKind, Read, Write};
use std::os::unix::io::AsRawFd;
use std::process::Command;
use std::time::{Duration, Instant};

pub trait Transport {
    /// Monotonic time since some fixed start.  (Simulated transports use a virtual clock, so tests don't wait.)
    fn now(&self) -> Duration;
    /// One byte, waiting at most `timeout`.  `Ok(None)` is a timeout (a zero or expired timeout never blocks).
    fn read_byte(&mut self, timeout: Duration) -> Result<Option<u8>, String>;
    /// Write everything and wait until it has really been sent.
    fn write_all(&mut self, data: &[u8]) -> Result<(), String>;
    /// Discard anything received but not yet read.
    fn flush_rx(&mut self) -> Result<(), String>;
    /// Change the host side's baud rate (draining output first and discarding input afterwards, like gpsctl).
    fn set_baud(&mut self, baud: u32) -> Result<(), String>;
    fn baud(&self) -> u32;
    /// Do nothing for `d` (simulated transports just advance their virtual clock).
    fn pause(&mut self, d: Duration) { std::thread::sleep(d) }
    /// After a host baud change while the receiver keeps sending, a UART can't find character boundaries until the line
    /// goes idle: everything up to the next gap is garbage, and so is the first character after it (a reply to a poll
    /// sent at once loses its B5 sync byte).  Discard input until the line has been quiet for `quiet`, but no longer
    /// than `max` altogether (waitForQuietLine() in gpsctl).
    fn wait_quiet(&mut self, quiet: Duration, max: Duration) -> Result<(), String> {
        let start = self.now();
        while self.now().saturating_sub(start) < max {
            if self.read_byte(quiet)?.is_none() {
                return Ok(());
            }
        }
        Ok(())
    }
    /// Has the port been hung up (device unplugged or reset, far end of a pty closed)?  Such a port never produces
    /// data again and refuses every ioctl: the only way back is `reopen` (portHungUp() in gpsctl).
    fn hung_up(&self) -> bool { false }
    /// Close the port and open the device again, waiting up to `wait` (looking every `look`) for it to come back
    /// (reopenSerialPort() in gpsctl).  Anything we knew about the link is stale afterwards.
    fn reopen(&mut self, _wait: Duration, _look: Duration) -> Result<(), String> { Err("this transport cannot be reopened".to_string()) }
}

/// How long the line must be quiet, and the most we wait for that, after a host baud change (QUIET_LINE_MS in gpsctl).
pub const QUIET_LINE: Duration = Duration::from_millis(20);
pub const QUIET_LINE_MAX: Duration = Duration::from_millis(1000);

/// What a dead port is called in error messages (same words as gpsctl).
pub const HUNG_UP_MSG: &str = "serial port hung up (device disconnected?)";

impl Transport for Box<dyn Transport> {
    fn now(&self) -> Duration { (**self).now() }
    fn read_byte(&mut self, timeout: Duration) -> Result<Option<u8>, String> { (**self).read_byte(timeout) }
    fn write_all(&mut self, data: &[u8]) -> Result<(), String> { (**self).write_all(data) }
    fn flush_rx(&mut self) -> Result<(), String> { (**self).flush_rx() }
    fn set_baud(&mut self, baud: u32) -> Result<(), String> { (**self).set_baud(baud) }
    fn baud(&self) -> u32 { (**self).baud() }
    fn pause(&mut self, d: Duration) { (**self).pause(d) }
    fn wait_quiet(&mut self, quiet: Duration, max: Duration) -> Result<(), String> { (**self).wait_quiet(quiet, max) }
    fn hung_up(&self) -> bool { (**self).hung_up() }
    fn reopen(&mut self, wait: Duration, look: Duration) -> Result<(), String> { (**self).reopen(wait, look) }
}

/// Wraps any transport and prints everything that crosses it to stderr: bytes sent, bytes received (grouped), flushes,
/// host baud changes.  For finding out what really happened on a port.
pub struct Traced<T: Transport> {
    pub inner: T,
    line: Vec<u8>,
}

impl<T: Transport> Traced<T> {
    pub fn new(inner: T) -> Traced<T> {
        Traced { inner, line: Vec::new() }
    }

    fn show(bytes: &[u8]) -> String {
        let hex: Vec<String> = bytes.iter().map(|b| format!("{b:02X}")).collect();
        let text: String = bytes.iter().map(|&b| if (32..127).contains(&b) { b as char } else { '.' }).collect();
        format!("{}  |{}|", hex.join(" "), text)
    }

    fn flush_line(&mut self) {
        if !self.line.is_empty() {
            eprintln!("[{:8.3}] RX {:3} bytes: {}", self.inner.now().as_secs_f64(), self.line.len(), Self::show(&self.line));
            self.line.clear();
        }
    }
}

impl<T: Transport> Transport for Traced<T> {
    fn now(&self) -> Duration { self.inner.now() }
    fn pause(&mut self, d: Duration) { self.inner.pause(d) }
    fn wait_quiet(&mut self, quiet: Duration, max: Duration) -> Result<(), String> {
        self.flush_line();
        eprintln!("[{:8.3}] wait for the line to go quiet ({} ms, at most {} ms)", self.inner.now().as_secs_f64(), quiet.as_millis(), max.as_millis());
        self.inner.wait_quiet(quiet, max)
    }

    fn read_byte(&mut self, timeout: Duration) -> Result<Option<u8>, String> {
        let r = self.inner.read_byte(timeout);
        match &r {
            Ok(Some(b)) => {
                self.line.push(*b);
                if self.line.len() >= 32 { self.flush_line(); }
            }
            Ok(None) => {
                self.flush_line();
                eprintln!("[{:8.3}] (nothing received for {} ms)", self.inner.now().as_secs_f64(), timeout.as_millis());
            }
            Err(_) => self.flush_line(),
        }
        r
    }

    fn write_all(&mut self, data: &[u8]) -> Result<(), String> {
        self.flush_line();
        eprintln!("[{:8.3}] TX {:3} bytes: {}", self.inner.now().as_secs_f64(), data.len(), Self::show(data));
        self.inner.write_all(data)
    }

    fn flush_rx(&mut self) -> Result<(), String> {
        self.flush_line();
        eprintln!("[{:8.3}] flush receive buffer", self.inner.now().as_secs_f64());
        self.inner.flush_rx()
    }

    fn set_baud(&mut self, baud: u32) -> Result<(), String> {
        self.flush_line();
        eprintln!("[{:8.3}] set host baud {baud}", self.inner.now().as_secs_f64());
        self.inner.set_baud(baud)
    }

    fn baud(&self) -> u32 { self.inner.baud() }
    fn hung_up(&self) -> bool { self.inner.hung_up() }

    fn reopen(&mut self, wait: Duration, look: Duration) -> Result<(), String> {
        self.flush_line();
        eprintln!("[{:8.3}] reopen port (waiting up to {} ms)", self.inner.now().as_secs_f64(), wait.as_millis());
        self.inner.reopen(wait, look)
    }
}

/// Baud rates gpsctl accepts (its `validRates` table).
pub const VALID_BAUDS: [u32; 20] = [
    50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600,
];

#[repr(C)]
struct PollFd {
    fd: i32,
    events: i16,
    revents: i16,
}

extern "C" {
    fn tcflush(fd: i32, queue_selector: i32) -> i32;
    fn tcdrain(fd: i32) -> i32;
    fn tcgetattr(fd: i32, termios: *mut u8) -> i32;
    fn poll(fds: *mut PollFd, nfds: u64, timeout_ms: i32) -> i32;
}
const TCIFLUSH: i32 = 0; // Linux value
const POLLIN: i16 = 0x001; // Linux values
const POLLERR: i16 = 0x008;
const POLLHUP: i16 = 0x010;
const POLLNVAL: i16 = 0x020;
const EINTR: i32 = 4;
const EIO: i32 = 5;
const ENODEV: i32 = 19;
const EPIPE: i32 = 32;

/// portHungUp(): poll() reports a hangup, a fatal error or a bad descriptor at once, even when asked for nothing; a
/// hung-up port also refuses every ioctl with EIO, so ask it directly as well.  A failed poll() is never read as "the
/// port is fine".
fn fd_hung_up(fd: i32) -> bool {
    let mut pfd = PollFd { fd, events: 0, revents: 0 };
    let mut pr;
    loop {
        pr = unsafe { poll(&mut pfd, 1, 0) };
        if pr >= 0 || std::io::Error::last_os_error().raw_os_error() != Some(EINTR) { break; }
    }
    if pr >= 0 && pfd.revents & (POLLERR | POLLHUP | POLLNVAL) != 0 {
        return true;
    }
    let mut termios = [0u8; 128]; // bigger than any struct termios
    unsafe { tcgetattr(fd, termios.as_mut_ptr()) != 0 }
}

/// The error text for a failed read, write or drain: a dead port is named plainly (it looks like a baud rate or
/// protocol problem otherwise), anything else keeps the system's words.
fn io_error(what: &str, e: &std::io::Error) -> String {
    match e.raw_os_error() {
        Some(EIO) | Some(EPIPE) | Some(ENODEV) => HUNG_UP_MSG.to_string(),
        _ => format!("{what} failed: {e}"),
    }
}

/// Other processes that have `path` open right now: (pid, command name).  Found by looking through /proc/*/fd, so a
/// process of another user is only seen when running as root (the caller can't tell "none" from "couldn't look" for
/// those; run with sudo, as the serial port usually needs anyway).  Our own process is never listed.
pub fn other_users(path: &str) -> Vec<(u32, String)> {
    let Ok(target) = std::fs::canonicalize(path) else { return vec![] };
    let me = std::process::id();
    let mut found = vec![];
    let Ok(procs) = std::fs::read_dir("/proc") else { return found };
    for p in procs.flatten() {
        let Some(pid) = p.file_name().to_str().and_then(|n| n.parse::<u32>().ok()) else { continue };
        if pid == me { continue; }
        let Ok(fds) = std::fs::read_dir(p.path().join("fd")) else { continue };
        let holds = fds.flatten().any(|fd| std::fs::read_link(fd.path()).map(|l| l == target).unwrap_or(false));
        if holds {
            let name = std::fs::read_to_string(p.path().join("comm")).unwrap_or_default().trim().to_string();
            found.push((pid, name));
        }
    }
    found.sort();
    found
}

pub struct SttyPort {
    file: File,
    path: String,
    baud: u32,
    pending: VecDeque<u8>,
    start: Instant,
}

fn stty(path: &str, args: &[&str]) -> Result<(), String> {
    // stderr is captured, not shown: a reopen tries every 200 ms while the device is away, and stty complaining each
    // time would bury the one message that matters.  The reason goes into the error text instead.
    let out = Command::new("stty").arg("-F").arg(path).args(args).output().map_err(|e| format!("can't run stty: {e}"))?;
    if out.status.success() {
        Ok(())
    } else {
        let why = String::from_utf8_lossy(&out.stderr).trim().to_string();
        Err(format!("stty failed on {path} (wrong port, or no permission - try sudo or the dialout group){}", if why.is_empty() { String::new() } else { format!(": {why}") }))
    }
}

impl SttyPort {
    /// Open once, for reading AND writing (a serial port discards incoming bytes while nothing has it open).  8N1, raw,
    /// no echo, no hangup on close; `min 0 time 1` makes a read return after 0.1 s even with no data, so deadlines work.
    /// Like `open_shared`, but refuses when another process already has the port open: two programs on one port steal
    /// each other's replies and fight over the baud rate (a running gpsd even answered our synchronizer's questions
    /// itself, giving a false "synchronized").
    pub fn open(path: &str, baud: u32) -> Result<SttyPort, String> {
        let others = other_users(path);
        if !others.is_empty() {
            let list: Vec<String> = others.iter().map(|(p, n)| format!("{n} (pid {p})")).collect();
            return Err(format!(
                "{path} is already open in: {}. Stop it first (e.g. `sudo systemctl stop gpsd.socket gpsd`), or pass --force to use the port anyway (results will be unreliable).",
                list.join(", ")
            ));
        }
        SttyPort::open_shared(path, baud)
    }

    pub fn open_shared(path: &str, baud: u32) -> Result<SttyPort, String> {
        if !VALID_BAUDS.contains(&baud) {
            return Err(format!("{baud} is not a valid baud rate"));
        }
        // open first: a device that is not there (or not allowed) is best reported in the system's own words, as the C
        // program does, rather than as a failure of stty
        let file = OpenOptions::new().read(true).write(true).open(path).map_err(|e| {
            let text = e.to_string();
            text.split(" (os error").next().unwrap_or(&text).to_string()
        })?;
        stty(path, &[&baud.to_string(), "raw", "-echo", "-hupcl", "clocal", "cs8", "-cstopb", "-parenb", "min", "0", "time", "1"])?;
        Ok(SttyPort { file, path: path.to_string(), baud, pending: VecDeque::new(), start: Instant::now() })
    }
}

impl Transport for SttyPort {
    fn now(&self) -> Duration {
        self.start.elapsed()
    }

    fn read_byte(&mut self, timeout: Duration) -> Result<Option<u8>, String> {
        if let Some(b) = self.pending.pop_front() {
            return Ok(Some(b));
        }
        let deadline = Instant::now() + timeout;
        let mut tmp = [0u8; 512];
        while Instant::now() < deadline {
            match self.file.read(&mut tmp) {
                Ok(0) => {
                    // the 0.1 s read timer expired with nothing received... or the kernel has hung the port up, which
                    // also reads as zero bytes, but at once and forever: poll() tells them apart
                    if fd_hung_up(self.file.as_raw_fd()) {
                        return Err(HUNG_UP_MSG.to_string());
                    }
                }
                Ok(n) => {
                    self.pending.extend(&tmp[..n]);
                    return Ok(self.pending.pop_front());
                }
                Err(e) if e.kind() == ErrorKind::Interrupted || e.kind() == ErrorKind::WouldBlock => {}
                Err(e) => return Err(io_error("read", &e)),
            }
        }
        Ok(None)
    }

    fn wait_quiet(&mut self, quiet: Duration, max: Duration) -> Result<(), String> {
        // poll() gives a finer quiet period than the 0.1 s read timer
        self.pending.clear();
        let start = Instant::now();
        let mut tmp = [0u8; 256];
        let fd = self.file.as_raw_fd();
        while start.elapsed() < max {
            let mut pfd = PollFd { fd, events: POLLIN, revents: 0 };
            let pr = unsafe { poll(&mut pfd, 1, quiet.as_millis() as i32) };
            if pr < 0 {
                let e = std::io::Error::last_os_error();
                if e.raw_os_error() == Some(EINTR) { continue; }
                return Err(io_error("poll", &e));
            }
            if pr == 0 { return Ok(()); }
            if pfd.revents & (POLLERR | POLLHUP | POLLNVAL) != 0 && pfd.revents & POLLIN == 0 {
                return Err(HUNG_UP_MSG.to_string());
            }
            match self.file.read(&mut tmp) {
                Ok(_) => {}
                Err(e) if e.kind() == ErrorKind::Interrupted || e.kind() == ErrorKind::WouldBlock => {}
                Err(e) => return Err(io_error("read", &e)),
            }
        }
        Ok(())
    }

    fn write_all(&mut self, data: &[u8]) -> Result<(), String> {
        self.file.write_all(data).and_then(|_| self.file.flush()).map_err(|e| io_error("write", &e))?;
        if unsafe { tcdrain(self.file.as_raw_fd()) } != 0 {
            return Err(io_error("tcdrain", &std::io::Error::last_os_error()));
        }
        Ok(())
    }

    fn flush_rx(&mut self) -> Result<(), String> {
        self.pending.clear();
        if unsafe { tcflush(self.file.as_raw_fd(), TCIFLUSH) } != 0 {
            return Err(io_error("tcflush", &std::io::Error::last_os_error()));
        }
        Ok(())
    }

    fn set_baud(&mut self, baud: u32) -> Result<(), String> {
        if !VALID_BAUDS.contains(&baud) {
            return Err(format!("{baud} is not a valid baud rate"));
        }
        if unsafe { tcdrain(self.file.as_raw_fd()) } != 0 {
            return Err(format!("tcdrain failed: {}", std::io::Error::last_os_error()));
        }
        stty(&self.path, &[&baud.to_string()])?;
        self.baud = baud;
        self.flush_rx()
    }

    fn baud(&self) -> u32 {
        self.baud
    }

    fn hung_up(&self) -> bool {
        fd_hung_up(self.file.as_raw_fd())
    }

    /// reopenSerialPort(): the old descriptor is dead for good, so open the device again, and only accept the new
    /// descriptor when it passes the same test (a node that is going away can linger long enough to be opened, and
    /// the port it hands back is just as dead).  Gives up after `wait`.
    fn reopen(&mut self, wait: Duration, look: Duration) -> Result<(), String> {
        let deadline = Instant::now() + wait;
        loop {
            let tried = SttyPort::open_shared(&self.path, self.baud);
            let last = match tried {
                Ok(fresh) if !fresh.hung_up() => {
                    *self = fresh; // drops (closes) the dead descriptor
                    return Ok(());
                }
                Ok(_) => "serial device is still hung up".to_string(),
                Err(e) => e,
            };
            if Instant::now() >= deadline {
                return Err(format!("serial device did not come back: {last}"));
            }
            std::thread::sleep(look);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn other_users_finds_a_process_holding_the_file_and_never_ourselves() {
        let path = std::env::temp_dir().join(format!("ubx_probe_holder_{}", std::process::id()));
        std::fs::write(&path, b"x").unwrap();
        let p = path.to_str().unwrap();
        let _mine = File::open(&path).unwrap(); // our own descriptor must not be reported
        assert_eq!(other_users(p), vec![]);
        let mut child = Command::new("sleep").arg("30").stdin(File::open(&path).unwrap()).spawn().unwrap();
        let found = other_users(p);
        child.kill().unwrap();
        child.wait().unwrap();
        std::fs::remove_file(&path).ok();
        assert_eq!(found, vec![(child.id(), "sleep".to_string())]);
    }

    #[test]
    fn open_refuses_a_port_someone_else_holds() {
        let path = std::env::temp_dir().join(format!("ubx_probe_busy_{}", std::process::id()));
        std::fs::write(&path, b"x").unwrap();
        let mut child = Command::new("sleep").arg("30").stdin(File::open(&path).unwrap()).spawn().unwrap();
        let err = SttyPort::open(path.to_str().unwrap(), 115200).err().expect("must refuse");
        child.kill().unwrap();
        child.wait().unwrap();
        std::fs::remove_file(&path).ok();
        assert!(err.contains("already open in: sleep (pid"), "{err}");
        assert!(err.contains("--force"), "{err}");
    }

    #[test]
    fn a_missing_path_has_no_other_users() {
        assert_eq!(other_users("/nonexistent/ttyXYZ"), vec![]);
    }

    // --- hangup detection, over a real pty pair ---

    extern "C" {
        fn posix_openpt(flags: i32) -> i32;
        fn grantpt(fd: i32) -> i32;
        fn unlockpt(fd: i32) -> i32;
        fn ptsname_r(fd: i32, buf: *mut u8, len: usize) -> i32;
        fn close(fd: i32) -> i32;
    }

    /// A pty: (master descriptor, slave path).  Closing the master hangs the slave up.
    fn pty() -> (i32, String) {
        unsafe {
            let m = posix_openpt(2 /* O_RDWR */ | 0o400 /* O_NOCTTY */ | 0o2000000 /* O_CLOEXEC: a stty child must not inherit the master and keep it open */);
            assert!(m >= 0 && grantpt(m) == 0 && unlockpt(m) == 0);
            let mut buf = [0u8; 128];
            assert_eq!(ptsname_r(m, buf.as_mut_ptr(), buf.len()), 0);
            let n = buf.iter().position(|&b| b == 0).unwrap();
            (m, String::from_utf8(buf[..n].to_vec()).unwrap())
        }
    }

    #[test]
    fn a_healthy_port_is_not_hung_up_and_a_quiet_one_just_times_out() {
        let (m, slave) = pty();
        let mut p = SttyPort::open_shared(&slave, 115200).unwrap();
        assert!(!p.hung_up());
        let t = Instant::now();
        assert_eq!(p.read_byte(Duration::from_millis(250)), Ok(None));
        assert!(t.elapsed() >= Duration::from_millis(240));
        unsafe { close(m) };
    }

    #[test]
    fn a_hung_up_port_is_reported_at_once_by_read_write_and_hung_up() {
        let (m, slave) = pty();
        let mut p = SttyPort::open_shared(&slave, 115200).unwrap();
        unsafe { close(m) }; // the far end goes away
        assert!(p.hung_up());
        let t = Instant::now();
        assert_eq!(p.read_byte(Duration::from_secs(5)), Err(HUNG_UP_MSG.to_string()));
        assert!(t.elapsed() < Duration::from_millis(500), "must not wait out the timeout: {:?}", t.elapsed());
        assert_eq!(p.write_all(&[1, 2, 3]), Err(HUNG_UP_MSG.to_string()));
    }

    #[test]
    fn bytes_already_read_are_still_delivered_after_the_hangup() {
        // (the kernel throws away what was still queued in the pty when the far end closed, so only what we had
        // already pulled in can be kept: nothing is lost on our side)
        let (m, slave) = pty();
        let mut p = SttyPort::open_shared(&slave, 115200).unwrap();
        let mut master = unsafe { <File as std::os::unix::io::FromRawFd>::from_raw_fd(m) };
        master.write_all(b"AB").unwrap();
        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(p.read_byte(Duration::from_millis(500)), Ok(Some(b'A'))); // B is now in our own queue
        drop(master);
        assert_eq!(p.read_byte(Duration::from_millis(500)), Ok(Some(b'B')));
        assert_eq!(p.read_byte(Duration::from_millis(500)), Err(HUNG_UP_MSG.to_string()));
    }

    #[test]
    fn reopen_waits_for_the_device_to_come_back() {
        let dir = std::env::temp_dir().join(format!("ubx_probe_reopen_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let link = dir.join("ttyX");
        let (m1, s1) = pty();
        std::os::unix::fs::symlink(&s1, &link).unwrap();
        let mut p = SttyPort::open_shared(link.to_str().unwrap(), 115200).unwrap();
        unsafe { close(m1) };
        assert!(p.hung_up());
        // the device is gone: nothing to open
        std::fs::remove_file(&link).unwrap();
        let t = Instant::now();
        let e = p.reopen(Duration::from_millis(400), Duration::from_millis(50)).unwrap_err();
        assert!(e.contains("did not come back"), "{e}");
        assert!(t.elapsed() >= Duration::from_millis(400));
        assert!(p.hung_up(), "a failed reopen leaves the old port in place");
        // ...and then it comes back after a moment, as a new pty
        let (m2, s2) = pty();
        let l2 = link.clone();
        let h = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(300));
            std::os::unix::fs::symlink(&s2, &l2).unwrap();
        });
        p.reopen(Duration::from_secs(3), Duration::from_millis(50)).unwrap();
        h.join().unwrap();
        assert!(!p.hung_up());
        assert_eq!(p.baud(), 115200);
        let mut master = unsafe { <File as std::os::unix::io::FromRawFd>::from_raw_fd(m2) };
        master.write_all(b"Z").unwrap();
        assert_eq!(p.read_byte(Duration::from_millis(500)), Ok(Some(b'Z')));
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn same_four_answers_as_the_c_port_hung_up() {
        // C (the real portHungUp, run on the same four cases): healthy 0, far end closed 1, fd -1 1, closed fd 1
        let (m, slave) = pty();
        let p = SttyPort::open_shared(&slave, 115200).unwrap();
        assert!(!p.hung_up());
        unsafe { close(m) };
        assert!(p.hung_up());
        assert!(fd_hung_up(-1));
        assert!(fd_hung_up(987_654)); // a descriptor nobody has open
    }
}
