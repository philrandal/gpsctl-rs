//! A simulated receiver behind `Transport`, with a virtual clock so tests never wait.  Test-only.
//! Behaviours (set by constructor): `stream_after_write` (bytes delivered once, after the first host write; baud-blind,
//! for poll golden tests), or a receiver at a fixed baud (`at_baud`) that speaks `Kind` when the host baud matches and
//! sends high-bit garbage when it doesn't.
#![cfg(test)]

use crate::cfg_nmea::{extract_frame, frame, parse_frame};
use crate::serial::Transport;
use std::collections::VecDeque;
use std::time::Duration;

#[derive(Clone, Copy, PartialEq)]
pub enum Kind { Nmea, Ubx }

pub struct Sim {
    pub clock: Duration,
    pub host_baud: u32,
    out: VecDeque<u8>,
    written: Vec<u8>,
    pub frames_seen: Vec<(u8, u8, Vec<u8>)>,
    /// Every byte the host has written, counted whether or not the receiver heard it (at the right rate)
    pub bytes_written: usize,
    stream: Option<Vec<u8>>,
    receiver: Option<(u32, Kind)>,
    junk: u8,
    nmea_pos: usize,
    /// CFG-PRT body served to polls, changed by sets when `stateful` (ubx receivers only)
    pub prt: Vec<u8>,
    pub stateful: bool,
}

pub const NMEA_LINE: &[u8] = b"$GNRMC,164025.00,A,5337.00390,N,00057.18741,W,0.026,,181126,,,A,V*0A\r\n";

impl Sim {
    fn base() -> Sim {
        Sim { clock: Duration::ZERO, host_baud: 921600, out: VecDeque::new(), written: vec![], frames_seen: vec![], bytes_written: 0, stream: None,
              receiver: None, junk: 0, nmea_pos: 0, prt: vec![], stateful: false }
    }
    /// Delivers `stream` once, as soon as the host has written anything.
    pub fn stream_after_write(stream: Vec<u8>) -> Sim {
        Sim { stream: Some(stream), ..Sim::base() }
    }
    pub fn at_baud(baud: u32, kind: Kind) -> Sim {
        Sim { receiver: Some((baud, kind)), ..Sim::base() }
    }
    fn char_time(&self) -> Duration {
        Duration::from_nanos(10 * (1_000_000_000 / self.host_baud as u64))
    }
    fn matched(&self) -> bool {
        matches!(self.receiver, Some((b, _)) if b == self.host_baud)
    }
    fn respond(&mut self) {
        while let Some(f) = extract_frame(&mut self.written) {
            let (c, i, b) = parse_frame(&f).unwrap();
            let body = b.to_vec();
            self.frames_seen.push((c, i, body.clone()));
            if !self.matched() { continue; }
            match (c, i) {
                (0x0A, 0x04) => self.out.extend(frame(0x0A, 0x04, &[b'A'; 40])),
                (0x06, 0x00) if body.len() == 1 => {
                    self.out.extend(frame(0x06, 0x00, &self.prt.clone()));
                    self.out.extend(frame(0x05, 0x01, &[0x06, 0x00]));
                }
                (0x06, 0x00) => {
                    // a set: ACK at the OLD baud like a real receiver, then (if it applies) switch rates
                    self.out.extend(frame(0x05, 0x01, &[0x06, 0x00]));
                    if self.stateful {
                        self.prt = body.clone();
                        let nb = u32::from_le_bytes([body[8], body[9], body[10], body[11]]);
                        self.receiver = Some((nb, Kind::Ubx));
                    }
                }
                _ => {}
            }
        }
    }
}

impl Transport for Sim {
    fn now(&self) -> Duration { self.clock }
    fn pause(&mut self, d: Duration) { self.clock += d; }

    fn read_byte(&mut self, timeout: Duration) -> Result<Option<u8>, String> {
        if let Some(b) = self.out.pop_front() {
            self.clock += self.char_time();
            return Ok(Some(b));
        }
        if let Some((_, kind)) = self.receiver {
            if !self.matched() {
                // wrong rate: endless high-bit junk, a character time apart
                if timeout < self.char_time() { self.clock += timeout; return Ok(None); }
                self.clock += self.char_time();
                self.junk = self.junk.wrapping_add(37);
                return Ok(Some(0x80 | self.junk));
            }
            if kind == Kind::Nmea {
                if timeout < self.char_time() { self.clock += timeout; return Ok(None); }
                self.clock += self.char_time();
                let b = NMEA_LINE[self.nmea_pos % NMEA_LINE.len()];
                self.nmea_pos += 1;
                return Ok(Some(b));
            }
        }
        self.clock += timeout;
        Ok(None)
    }

    fn write_all(&mut self, data: &[u8]) -> Result<(), String> {
        self.bytes_written += data.len();
        self.written.extend_from_slice(data);
        self.clock += self.char_time() * data.len() as u32;
        if let Some(s) = self.stream.take() {
            self.out.extend(s);
        }
        self.respond();
        Ok(())
    }

    fn flush_rx(&mut self) -> Result<(), String> {
        self.out.clear();
        Ok(())
    }

    fn set_baud(&mut self, baud: u32) -> Result<(), String> {
        self.host_baud = baud;
        self.out.clear();
        Ok(())
    }

    fn baud(&self) -> u32 { self.host_baud }
}

/// A receiver like c-golden/fake_any.py: for each configured (class, id) a body; a frame with a SHORTER body is a poll
/// (answered with the body and an ACK-ACK), anything else is a set (recorded, ACKed, and stored if `stateful`);
/// (class, id) pairs that aren't configured are never answered.
pub struct FakeAny {
    pub clock: Duration,
    out: VecDeque<u8>,
    written: Vec<u8>,
    pub bodies: std::collections::HashMap<(u8, u8), Vec<u8>>,
    pub stateful: bool,
    pub sets: Vec<Vec<u8>>,
}

impl FakeAny {
    pub fn new(bodies: std::collections::HashMap<(u8, u8), Vec<u8>>, stateful: bool) -> FakeAny {
        FakeAny { clock: Duration::ZERO, out: VecDeque::new(), written: vec![], bodies, stateful, sets: vec![] }
    }
    fn respond(&mut self) {
        while let Some(f) = extract_frame(&mut self.written) {
            let (c, i, b) = parse_frame(&f).unwrap();
            let Some(configured) = self.bodies.get(&(c, i)).cloned() else { continue };
            if b.len() < configured.len() {
                self.out.extend(frame(c, i, &configured));
                self.out.extend(frame(0x05, 0x01, &[c, i]));
            } else {
                self.sets.push(f.clone());
                self.out.extend(frame(0x05, 0x01, &[c, i]));
                if self.stateful {
                    self.bodies.insert((c, i), b.to_vec());
                }
            }
        }
    }
}

impl Transport for FakeAny {
    fn now(&self) -> Duration { self.clock }
    fn pause(&mut self, d: Duration) { self.clock += d; }
    fn read_byte(&mut self, timeout: Duration) -> Result<Option<u8>, String> {
        if let Some(b) = self.out.pop_front() {
            self.clock += Duration::from_nanos(10 * (1_000_000_000 / 115200));
            return Ok(Some(b));
        }
        self.clock += timeout;
        Ok(None)
    }
    fn write_all(&mut self, data: &[u8]) -> Result<(), String> {
        self.written.extend_from_slice(data);
        self.clock += Duration::from_nanos(10 * (1_000_000_000 / 115200)) * data.len() as u32;
        self.respond();
        Ok(())
    }
    fn flush_rx(&mut self) -> Result<(), String> { self.out.clear(); Ok(()) }
    fn set_baud(&mut self, _baud: u32) -> Result<(), String> { Ok(()) }
    fn baud(&self) -> u32 { 115200 }
}
