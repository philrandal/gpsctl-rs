//! UBX send / receive / poll / ACK over any `Transport`.  Mirrors gpsctl's rcvUbxMsg, ubxAckWait, sendUbxMsg and
//! pollUbx.  Differences, on purpose: frames are found with `cfg_nmea::extract_frame` (resync by one byte after a bad
//! checksum, and a body length over 4096 is treated as noise at once, as the C code now does too).

use crate::cfg_nmea::{extract_frame, frame, parse_frame};
use crate::cfg_prt::Link;
use crate::serial::Transport;
use std::time::Duration;

pub const ACK: u8 = 0x05;
pub const ACK_NAK: u8 = 0x00;
pub const ACK_ACK: u8 = 0x01;
/// How long to wait for the ACK of a set (ACK_WAIT_MS in the C code), and for the optional ACK after a CFG poll.
pub const ACK_WAIT_MS: u64 = 1500;
pub const POLL_ACK_DRAIN_MS: u64 = 250;
/// CFG-PRT polls get one second (CFG_PRT_MAX_MS-style budget used by pollUartBaudRate).
pub const CFG_POLL_MS: u64 = 1000;

#[derive(Debug, PartialEq, Clone)]
pub enum UbxError {
    /// No (good) message in time.
    Timeout,
    /// Receiver answered a poll with ACK-NAK.
    Nak,
    /// Time ran out after only unrelated messages were seen: the last one's class and id.
    Unexpected(u8, u8),
    Io(String),
}

impl std::fmt::Display for UbxError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            UbxError::Timeout => write!(f, "timed out waiting for a UBX message"),
            UbxError::Nak => write!(f, "received NAK"),
            UbxError::Unexpected(c, i) => write!(f, "received unexpected message type (0x{c:02x}-0x{i:02x}) in response to UBX poll"),
            UbxError::Io(e) => write!(f, "{e}"),
        }
    }
}

pub struct Ubx<T: Transport> {
    pub t: T,
    buf: Vec<u8>,
}

impl<T: Transport> Ubx<T> {
    pub fn new(t: T) -> Ubx<T> {
        Ubx { t, buf: Vec::new() }
    }

    pub fn flush_rx(&mut self) -> Result<(), String> {
        self.buf.clear();
        self.t.flush_rx()
    }

    /// See `Transport::hung_up`.
    pub fn hung_up(&self) -> bool {
        self.t.hung_up()
    }

    /// Reopen a dead port.  Half-received frames are stale, so they go too.
    pub fn reopen(&mut self, wait: Duration, look: Duration) -> Result<(), String> {
        self.buf.clear();
        self.t.reopen(wait, look)
    }

    pub fn set_baud(&mut self, baud: u32) -> Result<(), String> {
        self.buf.clear();
        self.t.set_baud(baud)
    }

    /// Extra time to allow on top of a fixed wait, for output the receiver has queued ahead of its reply: the time to
    /// receive `UBX_SYNC_BACKLOG_CHARS` characters at the port's current rate (lineBacklogMs() in gpsctl).
    pub fn backlog(&self) -> Duration {
        let baud = self.t.baud().max(1) as u64;
        let ns_char = 10 * (1_000_000_000 / baud);
        Duration::from_millis(ns_char * crate::sync::UBX_SYNC_BACKLOG_CHARS / 1_000_000)
    }

    pub fn send(&mut self, class: u8, id: u8, body: &[u8]) -> Result<(), UbxError> {
        self.t.write_all(&frame(class, id, body)).map_err(UbxError::Io)
    }

    /// The next good frame within `max`: (class, id, body).  Bad-checksum frames are skipped.
    pub fn receive(&mut self, max: Duration) -> Result<(u8, u8, Vec<u8>), UbxError> {
        let start = self.t.now();
        loop {
            if let Some(f) = extract_frame(&mut self.buf) {
                let (c, i, b) = parse_frame(&f).map_err(|e| UbxError::Io(format!("{e:?}")))?;
                return Ok((c, i, b.to_vec()));
            }
            let used = self.t.now().saturating_sub(start);
            if used >= max {
                return Err(UbxError::Timeout);
            }
            match self.t.read_byte(max - used).map_err(UbxError::Io)? {
                Some(b) => self.buf.push(b),
                None => return Err(UbxError::Timeout),
            }
        }
    }

    /// Wait for the ACK/NAK that names (class, id); ACKs for other messages and any other traffic are ignored.
    pub fn ack_wait(&mut self, class: u8, id: u8) -> Result<(), UbxError> {
        let total = Duration::from_millis(ACK_WAIT_MS) + self.backlog();
        self.ack_wait_for(class, id, total)
    }

    /// `ack_wait` for at most `total` (the baud change uses a short wait: that ACK proves nothing).
    pub fn ack_wait_for(&mut self, class: u8, id: u8, total: Duration) -> Result<(), UbxError> {
        let start = self.t.now();
        loop {
            let used = self.t.now().saturating_sub(start);
            if used >= total {
                return Err(UbxError::Timeout);
            }
            let (c, i, b) = self.receive(total - used)?;
            if c != ACK || b.len() < 2 || b[0] != class || b[1] != id {
                continue;
            }
            return if i == ACK_ACK { Ok(()) } else { Err(UbxError::Nak) };
        }
    }

    pub fn send_acked(&mut self, class: u8, id: u8, body: &[u8]) -> Result<(), UbxError> {
        self.send(class, id, body)?;
        self.ack_wait(class, id)
    }

    /// Poll: flush, send, then read until the reply of the same class/id arrives (stray messages and an early ACK-ACK
    /// are skipped); for CFG polls, also swallow the receiver's trailing ACK-ACK if it comes within 250 ms.
    pub fn poll(&mut self, class: u8, id: u8, body: &[u8], max_ms: u64) -> Result<Vec<u8>, UbxError> {
        let _ = self.flush_rx();
        self.send(class, id, body)?;
        let start = self.t.now();
        let total = Duration::from_millis(max_ms) + self.backlog();
        let mut got_ack = false;
        let mut unexpected: Option<(u8, u8)> = None;
        let answer = loop {
            let used = self.t.now().saturating_sub(start);
            if used >= total {
                return Err(match unexpected {
                    Some((c, i)) => UbxError::Unexpected(c, i),
                    None => UbxError::Timeout,
                });
            }
            let (c, i, b) = self.receive(total - used)?;
            if (c, i) == (class, id) {
                break b;
            }
            if (c, i) == (ACK, ACK_NAK) {
                return Err(UbxError::Nak);
            }
            if (c, i) == (ACK, ACK_ACK) {
                got_ack = true;
                continue;
            }
            unexpected = Some((c, i));
        };
        if class == 0x06 && !got_ack {
            // (queued output can delay the ACK at slow rates, as for every other wait)
            let wait = Duration::from_millis(POLL_ACK_DRAIN_MS) + self.backlog();
            let _ = self.receive(wait);
        }
        Ok(answer)
    }
}

/// So `cfg_prt::change_baud` can run over any transport.
impl<T: Transport> Link for Ubx<T> {
    fn poll_prt(&mut self) -> Option<Vec<u8>> {
        self.poll(0x06, 0x00, &[crate::cfg_prt::UART_PORT_ID], CFG_POLL_MS).ok()
    }
    fn send(&mut self, f: &[u8]) {
        let _ = self.t.write_all(f);
    }
    fn set_host_baud(&mut self, baud: u32) -> bool {
        Ubx::set_baud(self, baud).is_ok()
    }
    fn swallow_ack(&mut self) {
        // a moment only (POLL_ACK_DRAIN_MS, as ubxChangeBaudRate() does): the ACK is easily lost and the change is
        // confirmed by asking the receiver
        let _ = self.ack_wait_for(0x06, 0x00, Duration::from_millis(POLL_ACK_DRAIN_MS));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cfg_prt::{change_baud, BaudError};
    use crate::golden_util::hex;
    use crate::sim::{Kind, Sim};

    const GOLDEN_RXPOLL: &str = include_str!("../c-golden/golden_rxpoll.txt");

    fn result_text(r: Result<Vec<u8>, UbxError>) -> String {
        match r {
            Ok(b) if b.is_empty() => "ok".into(),
            Ok(b) => format!("ok {}", b.iter().map(|x| format!("{x:02X}")).collect::<Vec<_>>().join(" ")),
            Err(UbxError::Nak) => "nak".into(),
            Err(UbxError::Unexpected(c, i)) => format!("unexpected {c:02X}-{i:02X}"),
            Err(UbxError::Timeout) => "timeout".into(),
            Err(UbxError::Io(e)) => format!("io {e}"),
        }
    }

    #[test]
    fn poll_matches_the_c_implementation() {
        let mut n = 0;
        for line in GOLDEN_RXPOLL.lines() {
            let p: Vec<&str> = line.splitn(4, " : ").collect();
            let name = p[0].split_whitespace().nth(1).unwrap();
            let a: Vec<&str> = p[1].split_whitespace().collect();
            let (class, id) = (u8::from_str_radix(a[0], 16).unwrap(), u8::from_str_radix(a[1], 16).unwrap());
            let ms: u64 = a[2].parse().unwrap();
            let stream = if p[2] == "-" { vec![] } else { hex(p[2]) };
            let mut u = Ubx::new(Sim::stream_after_write(stream));
            let got = result_text(u.poll(class, id, &[], ms));
            assert_eq!(got, p[3], "{name}");
            n += 1;
        }
        assert_eq!(n, 20);
    }

    #[test]
    fn a_cfg_polls_trailing_ack_is_consumed_so_it_cannot_pass_for_a_later_sets_ack() {
        let stream = [frame(0x06, 0x00, &[1; 20]), frame(0x05, 0x01, &[0x06, 0x00])].concat();
        let mut u = Ubx::new(Sim::stream_after_write(stream));
        assert!(u.poll(0x06, 0x00, &[1], 300).is_ok());
        assert_eq!(u.ack_wait(0x06, 0x00), Err(UbxError::Timeout), "the poll's ACK must already be gone");
    }

    #[test]
    fn change_baud_over_a_simulated_receiver() {
        let prt = hex("01 00 00 00 C0 08 00 00 80 25 00 00 07 00 03 00 00 00 00 00"); // 9600
        let mut sim = Sim::at_baud(9600, Kind::Ubx);
        sim.prt = prt;
        sim.stateful = true;
        sim.host_baud = 9600;
        let mut u = Ubx::new(sim);
        assert_eq!(change_baud(&mut u, 115200), Ok(()));
        assert_eq!(u.t.host_baud, 115200);
        let sent: Vec<_> = u.t.frames_seen.iter().filter(|f| (f.0, f.1) == (0x06, 0x00) && f.2.len() == 20).collect();
        assert_eq!(sent.len(), 1);
        assert_eq!(&sent[0].2[8..12], &115200u32.to_le_bytes());
        assert_eq!(&sent[0].2[12..16], &hex("07 00 03 00")[..], "other fields untouched");
    }

    #[test]
    fn change_baud_that_the_receiver_ignores_puts_the_host_back() {
        let prt = hex("01 00 00 00 C0 08 00 00 80 25 00 00 07 00 03 00 00 00 00 00");
        let mut sim = Sim::at_baud(9600, Kind::Ubx);
        sim.prt = prt;
        sim.host_baud = 9600;
        let mut u = Ubx::new(sim);
        assert_eq!(change_baud(&mut u, 115200), Err(BaudError::NotChanged { wanted: 115200, still: 9600 }));
        assert_eq!(u.t.host_baud, 9600, "host must end where the receiver still is");
    }
}

#[cfg(test)]
mod backlog_tests {
    use super::*;
    use crate::sim::{Kind, Sim};

    #[test]
    fn waits_grow_with_the_line_time_like_the_c_code() {
        let mut u = Ubx::new(Sim::at_baud(9600, Kind::Ubx));
        u.t.host_baud = 9600;
        assert_eq!(u.backlog(), Duration::from_millis(1066)); // 1_041_660 ns/char * 1024
        u.t.host_baud = 115200;
        assert_eq!(u.backlog(), Duration::from_millis(88));
        // a poll nobody answers gives up after max_ms + backlog, not before
        u.t.host_baud = 9600;
        let t0 = u.t.now();
        assert_eq!(u.poll(0x01, 0x07, &[], 1000), Err(UbxError::Timeout));
        assert!(u.t.now() - t0 >= Duration::from_millis(2066));
    }
}
