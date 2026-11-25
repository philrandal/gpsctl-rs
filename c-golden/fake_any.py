#!/usr/bin/env python3
"""Generic fake receiver.  usage: fake_any.py <logfile> <CCII=bodyhex> ...   e.g. 0631=<32-byte body> 0624=<36-byte body>
A frame for class/id CCII whose body is SHORTER than the configured body is a poll: answered with the body + ACK-ACK.
Anything else for that class/id is a set: logged as 'SET <frame hex>' and ACKed (and, if FAKE_STATEFUL=1, stored, so later polls return it).
FAKE_BAUD=<n>: the receiver only hears the host at that baud rate (read from the pty's termios); a CFG-CFG that loads anything puts it back at 9600 (unless FAKE_NOSWITCH=1).
A key with the body "-" is never answered at all (the receiver stays silent).  Prints its pty slave path."""
import os, pty, select, sys, termios
SPEEDS = {getattr(termios, n): int(n[1:]) for n in dir(termios) if n.startswith('B') and n[1:].isdigit()}

def ck(b):
    a = c = 0
    for x in b:
        a = (a + x) & 255; c = (c + a) & 255
    return bytes([a, c])

def frame(cls, id_, body):
    h = bytes([0xB5, 0x62, cls, id_, len(body) & 255, len(body) >> 8]) + bytes(body)
    return h + ck(h[2:])

bodies = {bytes.fromhex(k): bytes.fromhex(v) for k, v in (a.split("=") for a in sys.argv[2:])}
stateful = os.environ.get("FAKE_STATEFUL") == "1"
master, slave = pty.openpty()
print(os.ttyname(slave), flush=True)
log = open(sys.argv[1], "w")
rx = bytearray()
rx_baud = int(os.environ.get("FAKE_BAUD", "0"))
silent = False          # FAKE_SILENT_AFTER_RST=1: once a CFG-RST has been seen the receiver never answers again
while True:
    r, _, _ = select.select([master], [], [], 0.2)
    if not r: continue
    heard = rx_baud == 0 or SPEEDS.get(termios.tcgetattr(master)[4], 0) == rx_baud
    try: data = os.read(master, 4096)
    except OSError: break
    if not heard: continue          # at the wrong rate it is only noise to the receiver
    rx += data
    while True:
        i = rx.find(b"\xb5\x62")
        if i < 0 or len(rx) < i + 8:
            if i > 0: del rx[:i]
            break
        del rx[:i]
        n = rx[4] | rx[5] << 8
        if len(rx) < 8 + n: break
        f = bytes(rx[:8 + n]); del rx[:8 + n]
        if ck(f[2:6 + n]) != f[6 + n:]: continue
        key = f[2:4]
        if key == b"\x06\x04" and os.environ.get("FAKE_SILENT_AFTER_RST") == "1":
            log.write("SET " + f.hex() + "\n"); log.flush(); silent = True
        if silent: continue
        if key not in bodies or bodies[key] == b"-": continue
        if n < len(bodies[key]):
            os.write(master, frame(key[0], key[1], bodies[key]) + frame(5, 1, key))
        else:
            log.write("SET " + f.hex() + "\n"); log.flush(); os.write(master, frame(5, 1, key))
            if key == b"\x06\x09" and rx_baud and int.from_bytes(f[14:18], "little") and os.environ.get("FAKE_NOSWITCH") != "1":
                rx_baud = 9600
            if stateful: bodies[key] = f[6:6 + n]
