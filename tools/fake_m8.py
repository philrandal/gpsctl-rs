#!/usr/bin/env python3
"""Fake receiver on a pty, to try nmea_tool without hardware.  Prints the slave path; run nmea_tool with --port <it>.
Answers CFG-NMEA polls with a real captured M8 reply, applies CFG-NMEA sets (ACK-ACK, or ACK-NAK on a bad checksum),
and interleaves NMEA text like a real receiver.   usage: fake_m8.py [seconds_to_run]"""
import os, pty, select, sys, time

def ck(b):
    a = c = 0
    for x in b:
        a = (a + x) & 255
        c = (c + a) & 255
    return bytes([a, c])

def frame(cls, id_, body):
    h = bytes([0xB5, 0x62, cls, id_, len(body) & 255, len(body) >> 8]) + bytes(body)
    return h + ck(h[2:])

body = bytearray(bytes.fromhex("0041000200000000000000010000000000000000"))
def padded(text, width):
    return text.encode().ljust(width, b"\0")[:width]

# made-up but plausibly shaped MON-VER body (NOT captured from a receiver)
verbody = padded("ROM CORE 3.01 (107888)", 30) + padded("00080000", 10) + b"".join(
    padded(e, 30) for e in ["FWVER=SPG 3.01", "PROTVER=18.00", "GPS;GLO;GAL;BDS", "SBAS;IMES;QZSS"])

gnssbody = bytes([0, 22, 22, 7]) + bytes.fromhex("0008100001000101" "0101030001000101" "0204080000000101" "0308100000000101" "0400080000000103" "0500030001000505" "06080E0001000101")  # sample, not captured

# made-up bodies for the other read-only commands (NOT captured from a receiver)
le = lambda v, n: v.to_bytes(n, "little")
msgbodies = {i: bytes([0xF0, i, 0, 1 if i in (0, 4) else 0, 0, 0, 0, 0]) for i in range(9)}   # GGA and RMC on
prtbody = bytes([1, 0, 0, 0]) + le(0x8D0, 4) + le(115200, 4) + le(7, 2) + le(3, 2) + le(0, 4)
ratebody = le(1000, 2) + le(1, 2) + le(1, 2)
pvt = bytearray(92)
pvt[4:6] = le(2026, 2); pvt[6:11] = bytes([11, 19, 0, 0, 0]); pvt[11] = 0x07; pvt[12:16] = le(30, 4)
pvt[16:20] = (-300).to_bytes(4, "little", signed=True); pvt[20] = 3; pvt[21] = 1; pvt[23] = 9
pvt[24:28] = (-12345678).to_bytes(4, "little", signed=True); pvt[28:32] = le(515000000, 4)
pvt[32:36] = le(60000, 4); pvt[36:40] = le(15000, 4); pvt[40:44] = le(2100, 4); pvt[44:48] = le(3200, 4)
sv = lambda g, i, cno, el, az, fl: bytes([g, i, cno, el & 255]) + le(az, 2) + le(0, 2) + le(fl, 4)
satbody = le(1000, 4) + bytes([1, 2, 0, 0]) + sv(0, 5, 44, 62, 271, 0x91C) + sv(6, 12, 0, 5, 359, 0x1001)

master, slave = pty.openpty()
print(os.ttyname(slave), flush=True)
end = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 30)
rx = bytearray()
noise = b"$GNRMC,164025.00,A,5337.00390,N,00057.18741,W,0.026,,181126,,,A,V*09\r\n"
last = 0
while time.time() < end:
    if time.time() - last > 0.2:
        os.write(master, noise); last = time.time()
    r, _, _ = select.select([master], [], [], 0.05)
    if not r:
        continue
    rx += os.read(master, 4096)
    while True:
        i = rx.find(b"\xb5\x62")
        if i < 0 or len(rx) < i + 8:
            if i > 0: del rx[:i]
            break
        del rx[:i]
        n = rx[4] | rx[5] << 8
        if len(rx) < 8 + n:
            break
        f = bytes(rx[:8 + n]); del rx[:8 + n]
        ok = ck(f[2:6 + n]) == f[6 + n:]
        if f[2:4] == b"\x06\x17" and n == 0 and ok:
            print("fake: poll", file=sys.stderr); os.write(master, frame(6, 0x17, body) + frame(5, 1, b"\x06\x17"))
        elif f[2:4] == b"\x06\x3e" and n == 0 and ok:
            print("fake: gnss poll", file=sys.stderr); os.write(master, frame(6, 0x3E, gnssbody) + frame(5, 1, b"\x06\x3e"))
        elif f[2:4] == b"\x0a\x04" and n == 0 and ok:
            print("fake: mon-ver poll", file=sys.stderr); os.write(master, frame(0x0A, 0x04, verbody))
        elif f[2:4] == b"\x06\x01" and n == 2 and ok and f[6] == 0xF0 and f[7] in msgbodies:
            print("fake: cfg-msg poll", file=sys.stderr); os.write(master, frame(6, 1, msgbodies[f[7]]) + frame(5, 1, b"\x06\x01"))
        elif f[2:4] == b"\x06\x00" and n == 1 and ok:
            print("fake: cfg-prt poll", file=sys.stderr); os.write(master, frame(6, 0, prtbody) + frame(5, 1, b"\x06\x00"))
        elif f[2:4] == b"\x06\x00" and n == 20 and ok:
            print("fake: cfg-prt set ->", int.from_bytes(f[14:18], "little"), file=sys.stderr); prtbody = f[6:6 + n]; os.write(master, frame(5, 1, b"\x06\x00"))
        elif f[2:4] == b"\x06\x08" and n == 0 and ok:
            print("fake: cfg-rate poll", file=sys.stderr); os.write(master, frame(6, 8, ratebody) + frame(5, 1, b"\x06\x08"))
        elif f[2:4] == b"\x01\x07" and n == 0 and ok:
            print("fake: nav-pvt poll", file=sys.stderr); os.write(master, frame(1, 7, pvt))
        elif f[2:4] == b"\x01\x35" and n == 0 and ok:
            print("fake: nav-sat poll", file=sys.stderr); os.write(master, frame(1, 0x35, satbody))
        elif f[2:4] == b"\x06\x17" and n == 20 and ok:
            print("fake: set ->", f[7:8].hex(), file=sys.stderr); body[:] = f[6:6 + n]; os.write(master, frame(5, 1, b"\x06\x17"))
        else:
            print("fake: bad/unknown frame", f.hex(), file=sys.stderr); os.write(master, frame(5, 0, b"\x06\x17"))
