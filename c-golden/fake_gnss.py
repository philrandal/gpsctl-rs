#!/usr/bin/env python3
"""Fake receiver for the CFG-GNSS / CFG-NMEA golden run.  usage: fake_gnss.py <gnss-body-hex> <nmea-body-hex> <log-file>
Prints its pty slave path, answers polls of CFG-GNSS (06 3E) and CFG-NMEA (06 17) with the given bodies (+ ACK-ACK, like a
real receiver), and logs every CFG set it receives as 'GNSS <frame hex>' / 'NMEA <frame hex>', ACKing each."""
import os, pty, select, sys

def ck(b):
    a = c = 0
    for x in b:
        a = (a + x) & 255; c = (c + a) & 255
    return bytes([a, c])

def frame(cls, id_, body):
    h = bytes([0xB5, 0x62, cls, id_, len(body) & 255, len(body) >> 8]) + bytes(body)
    return h + ck(h[2:])

gnss, nmea, logpath = bytes.fromhex(sys.argv[1]), bytes.fromhex(sys.argv[2]), sys.argv[3]
master, slave = pty.openpty()
print(os.ttyname(slave), flush=True)
log = open(logpath, "w")
rx = bytearray()
ack = lambda cls, id_: frame(5, 1, bytes([cls, id_]))
while True:
    r, _, _ = select.select([master], [], [], 0.2)
    if not r:
        continue
    try:
        rx += os.read(master, 4096)
    except OSError:
        break
    while True:
        i = rx.find(b"\xb5\x62")
        if i < 0 or len(rx) < i + 8:
            if i > 0: del rx[:i]
            break
        del rx[:i]
        n = rx[4] | rx[5] << 8
        if len(rx) < 8 + n: break
        f = bytes(rx[:8 + n]); del rx[:8 + n]
        if ck(f[2:6 + n]) != f[6 + n:]:
            continue
        key = f[2:4]
        if key == b"\x06\x3e" and n == 0:
            os.write(master, frame(6, 0x3E, gnss) + ack(6, 0x3E))
        elif key == b"\x06\x17" and n == 0:
            os.write(master, frame(6, 0x17, nmea) + ack(6, 0x17))
        elif key == b"\x06\x3e":
            log.write("GNSS " + f.hex() + "\n"); log.flush(); os.write(master, ack(6, 0x3E))
        elif key == b"\x06\x17":
            log.write("NMEA " + f.hex() + "\n"); log.flush(); os.write(master, ack(6, 0x17))
