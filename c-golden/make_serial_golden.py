#!/usr/bin/env python3
"""Golden data for the serial layer, from the REAL C functions:
  rxpoll <name> : <class> <id> <ms> : <stream hex sent after the poll> : <ok BODYHEX | nak | timeout | unexpected CC-II | senderr>
      (misc_harness `rx` = pollUbx against fake_stream.py)
  autobaud <name> : <method> <minBaud> : <receiver baud> <nmea|ubx> : <BAUD n | FAIL>
      (sync_harness = autoBaudRate against fake_baud.py, which only talks at its own baud)
usage: make_serial_golden.py <misc_h> <sync_h> <dir with the fake scripts> [rxpoll|autobaud]"""
import subprocess, sys, time
MISC, SYNC, D = sys.argv[1:4]
ONLY = sys.argv[4] if len(sys.argv) > 4 else None

def ck(b):
    a = c = 0
    for x in b: a = (a + x) & 255; c = (c + a) & 255
    return bytes([a, c])
def frame(cls, id_, body, bad=False):
    h = bytes([0xB5, 0x62, cls, id_, len(body) & 255, len(body) >> 8]) + bytes(body)
    c = ck(h[2:])
    if bad: c = bytes([c[0] ^ 0xFF, c[1]])
    return h + c
def hx(b): return " ".join(f"{x:02X}" for x in b)

def with_fake(script_args, cmd):
    fake = subprocess.Popen([sys.executable, "-I", D + "/" + script_args[0]] + script_args[1:], stdout=subprocess.PIPE, text=True)
    port = fake.stdout.readline().strip()
    try: return subprocess.run([a.replace("PORT", port) for a in cmd], capture_output=True, text=True, timeout=200)
    finally: fake.kill(); fake.wait()

def rxpoll(name, cls, id_, ms, stream):
    out = with_fake(["fake_stream.py", stream.hex() or "-"], [MISC, "PORT", "rx", f"{cls:02x}", f"{id_:02x}", str(ms)])
    body = [l for l in out.stdout.splitlines() if l.startswith("POLLBODY")]
    err = out.stderr
    if body and "RESULT ok" in out.stdout: res = "ok" + body[0][len("POLLBODY"):]
    elif "received NAK" in err: res = "nak"
    elif "received unexpected message type" in err:
        import re; m = re.search(r"\(0x(..)-0x(..)\)", err); res = f"unexpected {m[1].upper()}-{m[2].upper()}"
    elif "Problem sending" in err: res = "senderr"
    else: res = "timeout"
    print(f"rxpoll {name} : {cls:02X} {id_:02X} {ms} : {hx(stream) or '-'} : {res}", flush=True)

def autobaud(name, method, minbaud, rxbaud, kind):
    out = with_fake(["fake_baud.py", str(rxbaud), kind], [SYNC, "PORT", method, str(minbaud)])
    res = [l for l in out.stdout.splitlines() if l.startswith(("BAUD", "FAIL"))]
    print(f"autobaud {name} : {method} {minbaud} : {rxbaud} {kind} : {res[0] if res else 'NORESULT'}", flush=True)

if ONLY in (None, "rxpoll"):
    VER = frame(0x0A, 0x04, b"V" * 40)
    NOISE = b"$GNRMC,164025.00,A,5337.00390,N,00057.18741,W,0.026,,181126,,,A,V*0A\r\n"
    PRT = bytes(range(20)); CFG = frame(0x06, 0x00, PRT); ACK = frame(0x05, 0x01, b"\x06\x00")
    ms = 300
    rxpoll("clean-reply", 0x0A, 0x04, ms, VER)
    rxpoll("nmea-noise-then-reply", 0x0A, 0x04, ms, NOISE + NOISE + VER)
    rxpoll("bad-checksum-then-good", 0x0A, 0x04, ms, frame(0x0A, 0x04, b"X" * 40, bad=True) + VER)
    rxpoll("sync-pair-inside-noise", 0x0A, 0x04, ms, b"abc\xb5\x62zz\xb5" + VER)
    rxpoll("double-b5", 0x0A, 0x04, ms, b"\xb5" + VER)
    rxpoll("short-length-swallows-reply", 0x0A, 0x04, ms, bytes([0xB5, 0x62, 0x0A, 0x04, 0x0A, 0x00]) + VER)
    rxpoll("huge-length-then-reply", 0x0A, 0x04, ms, bytes([0xB5, 0x62, 0x0A, 0x04, 0x00, 0xFF]) + VER)
    rxpoll("length-4096-still-a-frame", 0x0A, 0x04, ms, bytes([0xB5, 0x62, 0x0A, 0x04, 0x00, 0x10]) + VER)
    rxpoll("length-4097-is-noise", 0x0A, 0x04, ms, bytes([0xB5, 0x62, 0x0A, 0x04, 0x01, 0x10]) + VER)
    rxpoll("unrelated-then-reply", 0x0A, 0x04, ms, frame(0x01, 0x07, b"\0" * 92) + frame(0x05, 0x01, b"\x06\x17") + VER)
    rxpoll("only-unrelated", 0x0A, 0x04, ms, frame(0x01, 0x07, b"\0" * 92))
    rxpoll("nothing", 0x0A, 0x04, ms, b"")
    rxpoll("truncated-reply", 0x0A, 0x04, ms, VER[:20])
    rxpoll("nak", 0x06, 0x00, ms, frame(0x05, 0x00, b"\x06\x00"))
    rxpoll("cfg-reply-then-ack", 0x06, 0x00, ms, CFG + ACK)
    rxpoll("cfg-ack-then-reply", 0x06, 0x00, ms, ACK + CFG)
    rxpoll("cfg-reply-no-ack", 0x06, 0x00, ms, CFG)
    rxpoll("stale-ack-for-other-then-reply", 0x06, 0x00, ms, frame(0x05, 0x01, b"\x06\x17") + CFG + ACK)
    rxpoll("empty-body-reply", 0x0A, 0x04, ms, frame(0x0A, 0x04, b""))
    rxpoll("two-replies-first-wins", 0x0A, 0x04, ms, VER + frame(0x0A, 0x04, b"W" * 40))

if ONLY in (None, "autobaud"):
    autobaud("ascii-nmea-115200", "ascii", 9600, 115200, "nmea")
    autobaud("nmea-nmea-460800", "nmea", 9600, 460800, "nmea")
    autobaud("ubx-ubx-230400", "ubx", 9600, 230400, "ubx")
    autobaud("ubx-ubx-921600-first-try", "ubx", 9600, 921600, "ubx")
    autobaud("ascii-nmea-below-min", "ascii", 115200, 57600, "nmea")
    autobaud("nmea-sync-on-ubx-only-receiver", "nmea", 460800, 115200, "ubx")
    autobaud("ascii-sync-on-ubx-only-receiver", "ascii", 460800, 115200, "ubx")
