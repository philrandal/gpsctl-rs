//! UBX CFG-PRT (class 0x06, id 0x00) for the UART (port id 1): the baud rate and whether NMEA output is on.  Mirrors
//! gpsctl's pollUartBaudRate(), sendUartBaudRate(), ubxChangeBaudRate() and isNmeaOn().
//!
//! `change_baud` is the whole verify-and-retry procedure, written against a small `Link` trait so it can be run with
//! no serial port at all (tests) or over a real one.  It never trusts an ACK: it re-polls the receiver, and only a poll
//! that reports the new rate counts.

use crate::cfg_nmea::frame;
use std::fmt;

pub const CLASS: u8 = 0x06;
pub const ID: u8 = 0x00;
pub const UART_PORT_ID: u8 = 1;
/// Bytes needed to read or write the baud rate (offset 8..12) the way gpsctl does: the full 20-byte UART body.
pub const MIN_BAUD_LEN: usize = 20;
/// outProtoMask is at offset 14..16; bit 1 is NMEA.
pub const MIN_PROTO_LEN: usize = 16;

pub fn poll_frame() -> Vec<u8> {
    frame(CLASS, ID, &[UART_PORT_ID])
}

#[derive(Debug, PartialEq)]
pub struct TooShort { pub needed: usize, pub got: usize }

pub fn baud(body: &[u8]) -> Result<u32, TooShort> {
    if body.len() < MIN_BAUD_LEN {
        return Err(TooShort { needed: MIN_BAUD_LEN, got: body.len() });
    }
    Ok(u32::from_le_bytes([body[8], body[9], body[10], body[11]]))
}

/// Is NMEA among the UART's output protocols?
pub fn nmea_out(body: &[u8]) -> Result<bool, TooShort> {
    if body.len() < MIN_PROTO_LEN {
        return Err(TooShort { needed: MIN_PROTO_LEN, got: body.len() });
    }
    Ok(u16::from_le_bytes([body[14], body[15]]) & 0x0002 != 0)
}

/// The set frame: the polled body with only the baud rate replaced (everything else, including trailing bytes, kept).
pub fn set_baud_frame(polled: &[u8], new_baud: u32) -> Result<Vec<u8>, TooShort> {
    baud(polled)?;
    let mut body = polled.to_vec();
    body[8..12].copy_from_slice(&new_baud.to_le_bytes());
    Ok(frame(CLASS, ID, &body))
}

#[cfg(test)]
pub(crate) fn tests_body(baud: u32) -> Vec<u8> {
    let mut b = vec![0u8; 20];
    b[0] = 1;
    b[8..12].copy_from_slice(&baud.to_le_bytes());
    b
}

/// What `change_baud` needs from the outside world.
pub trait Link {
    /// Poll CFG-PRT for the UART; the reply body, or None if there was no valid reply in time.
    fn poll_prt(&mut self) -> Option<Vec<u8>>;
    /// Send a complete frame (no ACK wait).
    fn send(&mut self, frame: &[u8]);
    /// Switch the host side of the serial port to this rate; false if the OS refused.
    fn set_host_baud(&mut self, baud: u32) -> bool;
    /// Best-effort wait for the ACK of the CFG-PRT just sent; the result is ignored.
    fn swallow_ack(&mut self) {}
}

#[derive(Debug, PartialEq)]
pub enum BaudError {
    /// Could not read the current configuration (no reply, or a body under 20 bytes).
    PollFailed,
    HostBaudRefused(u32),
    /// Neither attempt was confirmed, and the receiver says it is at a rate other than the wanted one.
    NotChanged { wanted: u32, still: u32 },
    /// Neither attempt was confirmed and the receiver then answered at neither the old nor the new rate.
    NoAnswer { wanted: u32, old: u32 },
}

impl fmt::Display for BaudError {
    // wording follows the C error messages
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            BaudError::PollFailed => write!(f, "Problem polling for current configuration before changing baud rate"),
            BaudError::HostBaudRefused(b) => write!(f, "Problem changing the host baud rate to {}", b),
            BaudError::NotChanged { wanted, still } => write!(f, "GPS did not change to {} baud (it is at {})", wanted, still),
            BaudError::NoAnswer { wanted, old } => {
                write!(f, "GPS did not change to {} baud (it answers at neither {} nor {})", wanted, old, wanted)
            }
        }
    }
}

fn poll_baud(link: &mut dyn Link) -> Option<u32> {
    link.poll_prt().and_then(|b| baud(&b).ok())
}

/// How many times the first read of the current rate is tried before giving up.
pub const INITIAL_POLL_TRIES: usize = 3;

/// Change the receiver's UART rate and the host's, and prove it worked (ubxChangeBaudRate()).
/// Up to two attempts; each asks up to twice for confirmation; after a failed attempt the host goes back to the old rate.
pub fn change_baud(link: &mut dyn Link, new_baud: u32) -> Result<(), BaudError> {
    change_baud_say(link, new_baud, &mut |_, _| {})
}

/// `change_baud`, also reporting what C prints along the way: `say(level, text)` is given the lowest verbosity at
/// which C would print the line (so the caller prints it when its verbosity is at least that).
pub fn change_baud_say(link: &mut dyn Link, new_baud: u32, say: &mut dyn FnMut(u8, String)) -> Result<(), BaudError> {
    // reading is harmless to repeat, and a glitch right after opening the port has been seen to garble one reply
    let old = (0..INITIAL_POLL_TRIES).find_map(|_| poll_baud(link)).ok_or(BaudError::PollFailed)?;
    // already there: nothing to send (rewriting the same configuration could only turn a lost ACK into a "failure")
    say(3, format!("GPS UART is currently at {old} baud..."));
    if old == new_baud {
        say(1, format!("GPS UART is already at {new_baud} baud..."));
        return Ok(());
    }
    for attempt in 0..2 {
        // sendUartBaudRate(): poll, change only the rate, send, move the host, swallow the ACK
        let polled = link.poll_prt().ok_or(BaudError::PollFailed)?;
        let f = set_baud_frame(&polled, new_baud).map_err(|_| BaudError::PollFailed)?;
        link.send(&f);
        if !link.set_host_baud(new_baud) {
            return Err(BaudError::HostBaudRefused(new_baud));
        }
        link.swallow_ack();

        // believe only what the receiver says it is using now
        let mut verified = false;
        for ask in 0..2 {
            if verified { break; }
            verified = poll_baud(link) == Some(new_baud);
            if !verified && ask == 0 {
                say(2, format!("No confirmation at {new_baud} baud, asking again..."));
            }
        }
        if verified {
            say(1, format!("Successfully changed baud rate to {new_baud}..."));
            return Ok(());
        }
        say(2, format!("GPS did not answer at {new_baud} baud, {}...", if attempt == 0 { "retrying" } else { "giving up" }));
        if !link.set_host_baud(old) {
            return Err(BaudError::HostBaudRefused(old));
        }
    }

    // Neither attempt was confirmed, but the receiver may still have taken the change and lost the replies.  The host
    // is back at the old rate (set again here, as C does): ask there first, then at the new rate, which keeps the host
    // where the receiver still answers whenever there is such a rate.
    if !link.set_host_baud(old) {
        return Err(BaudError::HostBaudRefused(old));
    }
    let mut probed = poll_baud(link);
    if probed.is_none() {
        if !link.set_host_baud(new_baud) {
            return Err(BaudError::HostBaudRefused(new_baud));
        }
        probed = poll_baud(link);
    }
    match probed {
        Some(b) if b == new_baud => {
            say(1, format!("Successfully changed baud rate to {new_baud}..."));
            Ok(())
        }
        Some(b) => Err(BaudError::NotChanged { wanted: new_baud, still: b }),
        None => Err(BaudError::NoAnswer { wanted: new_baud, old }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::golden_util::{cases, hex};

    /// A fake receiver like fake_any.py: a stored CFG-PRT body; a set stores it too when `stateful`.
    struct Fake { body: Vec<u8>, stateful: bool, sent: Vec<Vec<u8>>, host: Vec<u32>, polls: usize }

    impl Link for Fake {
        fn poll_prt(&mut self) -> Option<Vec<u8>> { self.polls += 1; Some(self.body.clone()) }
        fn send(&mut self, f: &[u8]) {
            self.sent.push(f.to_vec());
            if self.stateful { self.body = f[6..f.len() - 2].to_vec(); }
        }
        fn set_host_baud(&mut self, b: u32) -> bool { self.host.push(b); true }
    }

    #[test]
    fn poll_frame_is_the_documented_one() {
        assert_eq!(&poll_frame()[..7], &[0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01]);
    }

    #[test]
    fn nmea_flag_matches_the_c_implementation() {
        let cs = cases("nmeaon");
        assert_eq!(cs.len(), 5);
        for c in cs {
            match nmea_out(&c.body) {
                Ok(v) => {
                    assert!(c.ok(), "{}", c.name);
                    assert_eq!(c.find("NMEAON ").unwrap(), if v { "1" } else { "0" }, "{}", c.name);
                }
                Err(_) => assert!(!c.ok(), "{}", c.name),
            }
        }
    }

    #[test]
    fn change_baud_matches_the_c_implementation() {
        let cs = cases("baud");
        assert_eq!(cs.len(), 5);
        for c in cs {
            // the "takes" cases are run against a receiver that applies the change; the others against one that doesn't
            let stateful = c.name.contains("takes");
            let mut fake = Fake { body: c.body.clone(), stateful, sent: vec![], host: vec![], polls: 0 };
            let want: u32 = c.args[1].parse().unwrap();
            let r = change_baud(&mut fake, want);
            assert_eq!(r.is_ok(), c.ok(), "{}: {:?}", c.name, r);
            let c_sent: Vec<Vec<u8>> = c.all("SENT ").into_iter().map(hex).collect();
            assert_eq!(fake.sent, c_sent, "{}: frames sent", c.name);
            let c_host: Vec<u32> = c.all("HOSTBAUD ").iter().map(|s| s.parse().unwrap()).collect();
            assert_eq!(fake.host, c_host, "{}: host baud changes", c.name);
            if let Err(e) = r {
                assert_eq!(format!("{}", e), c.find("ERR: ").unwrap(), "{}", c.name);
            }
        }
    }

    #[test]
    fn the_first_read_is_retried_when_a_reply_is_garbled() {
        struct Flaky { inner: Fake, lose: usize }
        impl Link for Flaky {
            fn poll_prt(&mut self) -> Option<Vec<u8>> {
                if self.lose > 0 { self.lose -= 1; return None; }
                self.inner.poll_prt()
            }
            fn send(&mut self, f: &[u8]) { self.inner.send(f) }
            fn set_host_baud(&mut self, b: u32) -> bool { self.inner.set_host_baud(b) }
        }
        let body = hex("01 00 00 00 C0 08 00 00 00 C2 01 00 07 00 03 00 00 00 00 00");
        let mk = |lose| Flaky { inner: Fake { body: body.clone(), stateful: true, sent: vec![], host: vec![], polls: 0 }, lose };
        assert_eq!(change_baud(&mut mk(2), 230400), Ok(()), "two lost replies are survived");
        assert_eq!(change_baud(&mut mk(3), 230400), Err(BaudError::PollFailed), "three are not");
        assert!(mk(3).inner.sent.is_empty());
    }

    #[test]
    fn retry_recovers_when_the_first_confirmation_is_lost() {
        // receiver that stays deaf for the first poll after a set, as the C comment says it can
        struct Deaf { inner: Fake, deaf_polls: usize }
        impl Link for Deaf {
            fn poll_prt(&mut self) -> Option<Vec<u8>> {
                if self.deaf_polls > 0 && !self.inner.sent.is_empty() { self.deaf_polls -= 1; return None; }
                self.inner.poll_prt()
            }
            fn send(&mut self, f: &[u8]) { self.inner.send(f) }
            fn set_host_baud(&mut self, b: u32) -> bool { self.inner.set_host_baud(b) }
        }
        let body = hex("01 00 00 00 D0 08 00 00 80 25 00 00 07 00 03 00 00 00 00 00");
        let mut l = Deaf { inner: Fake { body, stateful: true, sent: vec![], host: vec![], polls: 0 }, deaf_polls: 1 };
        assert_eq!(change_baud(&mut l, 115200), Ok(()));
        assert_eq!(l.inner.sent.len(), 1, "second ask succeeds, so no resend");
        assert_eq!(l.inner.host, vec![115200]);
    }

    /// A receiver that answers `answers` (poll number, 1-based -> reported rate) and is silent otherwise.  Poll order of
    /// a failing change: 1 initial read, 2 before the first set, 3-4 confirmations, 5 before the second set, 6-7
    /// confirmations, 8 probe at the old rate, 9 probe at the new rate.
    struct Script { polls: usize, answers: Vec<(usize, u32)>, host: Vec<u32> }
    impl Link for Script {
        fn poll_prt(&mut self) -> Option<Vec<u8>> {
            self.polls += 1;
            self.answers.iter().find(|(n, _)| *n == self.polls).map(|(_, b)| crate::cfg_prt::tests_body(*b))
        }
        fn send(&mut self, _f: &[u8]) {}
        fn set_host_baud(&mut self, b: u32) -> bool { self.host.push(b); true }
    }

    #[test]
    fn already_at_the_rate_sends_nothing() {
        let mut l = Script { polls: 0, answers: vec![(1, 115200)], host: vec![] };
        assert_eq!(change_baud(&mut l, 115200), Ok(()));
        assert!(l.host.is_empty());
    }

    #[test]
    fn silent_receiver_is_reported_as_answering_at_neither() {
        let mut l = Script { polls: 0, answers: vec![(1, 9600), (2, 9600), (5, 9600)], host: vec![] };
        assert_eq!(change_baud(&mut l, 115200), Err(BaudError::NoAnswer { wanted: 115200, old: 9600 }));
        // two failed attempts (each puts the host back), the extra restore, then the probe at the new rate
        assert_eq!(l.host, vec![115200, 9600, 115200, 9600, 9600, 115200]);
    }

    #[test]
    fn replies_lost_but_change_took_is_success() {
        // the receiver switched but every confirmation was lost; the probe at the old rate is silent, the one at the new
        // rate answers, and the host is left at the new rate
        let mut l = Script { polls: 0, answers: vec![(1, 9600), (2, 9600), (5, 9600), (9, 115200)], host: vec![] };
        assert_eq!(change_baud(&mut l, 115200), Ok(()));
        assert_eq!(l.host.last(), Some(&115200));
    }

    #[test]
    fn receiver_still_at_the_old_rate_says_so() {
        let mut l = Script { polls: 0, answers: vec![(1, 9600), (2, 9600), (5, 9600), (8, 9600)], host: vec![] };
        assert_eq!(change_baud(&mut l, 115200), Err(BaudError::NotChanged { wanted: 115200, still: 9600 }));
        assert_eq!(l.host, vec![115200, 9600, 115200, 9600, 9600]); // left at the old rate, where it answers
    }
}
