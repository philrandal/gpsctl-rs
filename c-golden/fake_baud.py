#!/usr/bin/env python3
"""Fake receiver on a pty that cares about the host's baud rate (read from the pty's termios).
usage: fake_baud.py <receiver baud> <nmea|ubx>
 nmea: at the right rate, sends a valid NMEA sentence every 100 ms.  ubx: at the right rate, answers MON-VER polls.
 At any other rate it sends high-bit junk (what a real receiver's output looks like to a host at the wrong speed)."""
import os, pty, select, sys, termios, time
rx_baud, kind = int(sys.argv[1]), sys.argv[2]
SPEEDS = {getattr(termios, n): int(n[1:]) for n in dir(termios) if n.startswith("B") and n[1:].isdigit()}
def ck(b):
    a = c = 0
    for x in b:
        a = (a + x) & 255; c = (c + a) & 255
    return bytes([a, c])
def frame(cls, id_, body):
    h = bytes([0xB5, 0x62, cls, id_, len(body) & 255, len(body) >> 8]) + bytes(body)
    return h + ck(h[2:])
LINE = b"$GNRMC,164025.00,A,5337.00390,N,00057.18741,W,0.026,,181126,,,A,V*0A\r\n"
master, slave = pty.openpty()
print(os.ttyname(slave), flush=True)
rx = bytearray(); last = 0; junk = 0
while True:
    host = SPEEDS.get(termios.tcgetattr(master)[4], 0)
    match = host == rx_baud
    r, _, _ = select.select([master], [], [], 0.02)
    if r:
        rx += os.read(master, 4096)
        while True:
            i = rx.find(b"\xb5\x62")
            if i < 0 or len(rx) < i + 8: break
            del rx[:i]
            n = rx[4] | rx[5] << 8
            if len(rx) < 8 + n: break
            f = bytes(rx[:8 + n]); del rx[:8 + n]
            if match and kind == "ubx" and f[2:4] == b"\x0a\x04":
                os.write(master, frame(0x0A, 0x04, b"A" * 40))
        rx.clear() if len(rx) > 4096 else None
    if time.time() - last > 0.1:
        last = time.time()
        if not match:
            junk = (junk + 37) & 0x7F
            os.write(master, bytes((0x80 | ((junk + k * 11) & 0x7F)) for k in range(8)))
        elif kind == "nmea":
            os.write(master, LINE)
