#!/usr/bin/env python3
"""Fake receiver on a pty that models a receiver with a nearly full line at its own baud rate (the u-blox defaults at 9600),
and a UART that loses character alignment when the host changes baud rate mid-stream.
usage: fake_flood.py <logfile> <CCII=bodyhex> ...   env FAKE_BAUD=<receiver baud> (default 9600)
 - every second it sends a ~0.7 s burst of NMEA that keeps the line busy (chunks queued back to back); replies queue behind it
 - the host's rate is read from the pty's termios; if it differs from FAKE_BAUD everything is garbage (and nothing is heard)
 - a host baud change makes the UART misaligned: all output is garbled until the line has been idle for 5 ms
 - log: POLL <class><id> good|CORRUPT for every poll answered, SET ... for sets"""
import os, pty, select, sys, termios, time
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
rx_baud = int(os.environ.get("FAKE_BAUD", "9600"))
master, slave = pty.openpty()
print(os.ttyname(slave), flush=True)
log = open(sys.argv[1], "w")
LINE = (b"$GNRMC,152643.00,A,5337.00754,N,00057.18735,W,0.018,,221126,,,D*7D\r\n" * 2)[:67]
CHUNK = len(LINE)
rx = bytearray()
queue = []                 # (bytes, kind) waiting for the line
cur = bytearray()
chunk_times = []
busy_until = 0.0           # the line is occupied until then
misaligned = False
last_host = SPEEDS.get(termios.tcgetattr(master)[4], 0)
t0 = time.time()
next_chunk = t0 + 1.0
while True:
    now = time.time()
    host = SPEEDS.get(termios.tcgetattr(master)[4], 0)
    if host != last_host:
        last_host = host
        misaligned = True
        log.write(f'{time.time()-t0:7.3f} host baud {host} -> misaligned (queue {len(queue)})\n'); log.flush()
    # one burst a second: 12 chunks, one every 60 ms (each needs 70 ms of line time at 9600 baud)
    while now >= next_chunk:
        for k in range(12):
            chunk_times.append(next_chunk + k * 0.06)
        next_chunk += 1.0
    while chunk_times and chunk_times[0] <= now:
        chunk_times.pop(0)
        queue.append((LINE, "nmea"))
    # put the next item on the line, a few characters at a time as a UART would (so the host sees a continuous stream)
    if not cur and queue and now >= busy_until:
        data, kind = queue.pop(0)
        garbled = host != rx_baud or misaligned
        cur = bytearray(bytes(b ^ 0xA5 for b in data) if garbled else data)
        if kind != "nmea":
            log.write(f"{time.time()-t0:7.3f} POLL {kind} {'CORRUPT' if garbled else 'good'}\n"); log.flush()
    if cur and now >= busy_until:
        piece = bytes(cur[:4]); del cur[:4]
        os.write(master, piece)
        busy_until = max(now, busy_until) + len(piece) * 10.0 / rx_baud
    elif not cur and not queue and now >= busy_until + 0.005:
        if misaligned:
            log.write(f'{time.time()-t0:7.3f} line idle -> aligned\n'); log.flush()
        misaligned = False            # the UART has seen the line idle
    r, _, _ = select.select([master], [], [], 0.001)
    if not r: continue
    try: data = os.read(master, 4096)
    except OSError: break
    if SPEEDS.get(termios.tcgetattr(master)[4], 0) != rx_baud: continue
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
        if key not in bodies: continue
        if n < len(bodies[key]):
            queue.append((frame(key[0], key[1], bodies[key]) + frame(5, 1, key), key.hex()))
        else:
            log.write("SET " + f.hex() + "\n"); log.flush()
            queue.append((frame(5, 1, key), "ack"))
