#!/usr/bin/env python3
"""Golden data for the configuration commands: runs the REAL C functions (cmd_harness.c) against fake_any.py with a
gpsctl.conf, and prints one tab-separated line per case:
  name <TAB> command args <TAB> ini (newlines as \\n) <TAB> receiver bodies (CCII=hex ...) <TAB> stateful(0/1) <TAB> ok|error <TAB> warning count <TAB> frames sent, '|' separated
usage: make_cmd_golden.py <cmd_harness> <dir with fake_any.py>"""
import os, subprocess, sys, tempfile
H, G = sys.argv[1], sys.argv[2]

def le(v, n): return v.to_bytes(n, "little")
def hx(b): return bytes(b).hex().upper()

def run(args, ini, bodies, stateful):
    with tempfile.TemporaryDirectory() as d:
        lp, ip = d + "/log", d + "/gpsctl.conf"
        open(ip, "w").write(ini)
        env = dict(os.environ, FAKE_STATEFUL="1" if stateful else "0")
        fake = subprocess.Popen([sys.executable, "-I", G + "/fake_any.py", lp] + [f"{k}={v}" for k, v in bodies.items()],
                                stdout=subprocess.PIPE, text=True, env=env)
        port = fake.stdout.readline().strip()
        try: out = subprocess.run([H, port, ip] + args, capture_output=True, text=True, timeout=120)
        finally: fake.kill(); fake.wait()
        sets = [l.split()[1].upper() for l in open(lp).read().splitlines() if l.startswith("SET ")]
        text = out.stdout + out.stderr
        warns = sum(1 for l in text.splitlines() if l.startswith("Warning:"))
        res = "ok" if "RESULT ok" in out.stdout else "error"
        return res, warns, sets

def emit(name, args, ini, bodies, stateful=False):
    res, warns, sets = run(args, ini, bodies, stateful)
    b = " ".join(f"{k}={v}" for k, v in bodies.items())
    print("\t".join([name, " ".join(args), ini.replace("\n", "\\n"), b, "1" if stateful else "0", res, str(warns), "|".join(sets)]), flush=True)

# --- receiver bodies
PRT = lambda out=0x0003, n=20: (bytes([1, 0]) + le(0, 2) + le(0x8D0, 4) + le(115200, 4) + le(7, 2) + le(out, 2) + le(0, 2) + le(0, 2))[:n]
MSG = lambda rates=(0, 0, 0, 0, 0, 0), id_=0: bytes([0xF0, id_] + list(rates))
NMEA = lambda ver=0x40, n=20: (bytes([0, ver, 8, 0]) + bytes(range(16)))[:n]
def gnss(blocks):
    b = bytes([0, 32, 32, len(blocks)])
    for gid, mn, mx, flags in blocks: b += bytes([gid, mn, mx, 0]) + le(flags, 4)
    return b
GNSS = gnss([(0, 8, 16, 0x01010001), (1, 1, 3, 0x01010000), (2, 4, 8, 0x01010000), (3, 8, 16, 0x01010000),
             (4, 0, 8, 0x01010000), (5, 0, 3, 0x01010000), (6, 8, 14, 0x01010001)])
TP5 = bytes([0, 1, 0, 0]) + bytes(range(100, 128))       # 32 bytes, recognisable filler
NAV5 = bytes(range(1, 37))                               # 36 bytes
CFG = bytes(13)                                          # CFG-CFG body (a frame this long is a set)

def h(b): return hx(b)

MONVER = bytes(40)
RESTART = {"0609": h(CFG), "0A04": h(MONVER)}   # what a GNSS change with Galileo needs: CFG-CFG acked, MON-VER answered
SPRING = {"0609": h(CFG), "0604": ""}
ALL = {"0601": h(MSG()), "0617": h(NMEA()), "063E": h(GNSS), "0631": h(TP5), "0624": h(NAV5), "0600": h(PRT()), **RESTART}

# save / reset
emit("save-acked", ["save"], "", {"0609": h(CFG)})
emit("save-no-ack", ["save"], "", {})
emit("reset-sends", ["reset"], "", {"0604": ""})
# NMEA data on the UART
emit("nmeadata-turn-on", ["nmeadata", "1"], "", {"0600": h(PRT(0x0001))})
emit("nmeadata-turn-off", ["nmeadata", "0"], "", {"0600": h(PRT(0x0003))})
emit("nmeadata-already-on", ["nmeadata", "1"], "", {"0600": h(PRT(0x0003))})
emit("nmeadata-already-off", ["nmeadata", "0"], "", {"0600": h(PRT(0x0001))})
emit("nmeadata-body-18", ["nmeadata", "1"], "", {"0600": h(PRT(0x0001, 18))})
emit("nmeadata-body-15", ["nmeadata", "1"], "", {"0600": h(PRT(0x0001, 15))})
emit("nmeadata-silent", ["nmeadata", "1"], "", {})
# one NMEA message
emit("msg-enable", ["msg", "4", "1"], "", {"0601": h(MSG((2, 0, 3, 4, 5, 6), 4))})
emit("msg-already", ["msg", "4", "0"], "", {"0601": h(MSG((2, 0, 3, 4, 5, 6), 4))})
emit("msg-short-7", ["msg", "4", "1"], "", {"0601": h(MSG((2, 0, 3, 4, 5, 6), 4)[:7])})
# NMEA version
emit("nmeaver-change", ["nmeaver", "41"], "", {"0617": h(NMEA(0x40))})
emit("nmeaver-same", ["nmeaver", "41"], "", {"0617": h(NMEA(0x41))})
emit("nmeaver-short-3", ["nmeaver", "41"], "", {"0617": h(NMEA(0x40, 3))})
# satellites
emit("sat-defaults", ["sat"], "", {"063E": h(GNSS), "0617": h(NMEA(0x40)), **RESTART})
GNSS_SAME = gnss([(0, 8, 16, 0x01010001), (1, 1, 3, 0x01010000), (2, 4, 8, 0x01010001), (3, 8, 16, 0x01010000),
                  (4, 0, 8, 0x01010000), (5, 0, 3, 0x01010000), (6, 8, 14, 0x01010001)])
GNSS_NOGLO = gnss([(0, 8, 16, 0x01010001), (1, 1, 3, 0x01010000), (2, 4, 8, 0x01010001), (3, 8, 16, 0x01010000),
                   (4, 0, 8, 0x01010000), (5, 0, 3, 0x01010000), (6, 8, 14, 0x01010000)])
emit("sat-unchanged", ["sat"], "", {"063E": h(GNSS_SAME), "0617": h(NMEA(0x41))}, True)
emit("sat-glonass-off-in-receiver", ["sat"], "", {"063E": h(GNSS_NOGLO), "0617": h(NMEA(0x41)), **RESTART}, True)
emit("sat-galileo-off-in-config", ["sat"], "[galileo]\nenabled = no\n", {"063E": h(GNSS_SAME), "0617": h(NMEA(0x41))}, True)
emit("sat-restart-no-save-ack", ["sat"], "", {"063E": h(GNSS), "0617": h(NMEA(0x40)), "0A04": h(MONVER)})
emit("sat-restart-no-monver", ["sat"], "", {"063E": h(GNSS), "0617": h(NMEA(0x40)), "0609": h(CFG)})
emit("sat-custom", ["sat"], "[gps]\nenabled = yes\nmaximum channels = 12\n[glonass]\nenabled = no\n[beidou]\nenabled = yes\n[nmea]\nversion = 4.0\n", {"063E": h(GNSS), "0617": h(NMEA(0x23)), **RESTART})
emit("sat-bad-channels", ["sat"], "[gps]\nmaximum channels = 300\n[galileo]\nminimum channels = 6 channels\n", {"063E": h(GNSS), "0617": h(NMEA(0x41)), **RESTART})
emit("sat-no-nmea-reply", ["sat"], "", {"063E": h(GNSS)})
emit("sat-no-gnss-reply", ["sat"], "", {"0617": h(NMEA(0x40))})
emit("sat-nmea-text", ["sat"], "[nmea]\nversion = abc\n", {"063E": h(GNSS), "0617": h(NMEA(0x40)), **RESTART})
emit("sat-nmea-411", ["sat"], "[nmea]\nversion = 4.11\n", {"063E": h(GNSS), "0617": h(NMEA(0x40)), **RESTART})
emit("sat-nmea-411-digits", ["sat"], "[nmea]\nversion = 411\n", {"063E": h(GNSS), "0617": h(NMEA(0x40)), **RESTART})
emit("sat-nmea-bad-version", ["sat"], "[nmea]\nversion = 4.2\n", {"063E": h(GNSS), "0617": h(NMEA(0x40)), **RESTART})
emit("sat-nmea-23-no-galileo", ["sat"], "[galileo]\nenabled = no\n[nmea]\nversion = 2.3\n", {"063E": h(GNSS), "0617": h(NMEA(0x41)), **RESTART})
# time pulse
emit("tp-defaults", ["tp"], "", {"0631": h(TP5)})
emit("tp-custom", ["tp"], "[time pulse]\nantenna cable delay = -20 ; ns\nrf group delay = 0x10\nunlocked pulse period = 2000000\nunlocked pulse length = 5\nlocked pulse period = 3000000\nlocked pulse length = 7\nuser configurable delay = -100\n", {"0631": h(TP5)})
emit("tp-limits", ["tp"], "[time pulse]\nantenna cable delay = -32768\nrf group delay = 32767\nuser configurable delay = -2147483648\nlocked pulse length = 2147483647\n", {"0631": h(TP5)})
emit("tp-bad-values", ["tp"], "[time pulse]\nantenna cable delay = 56 nanoseconds\nrf group delay = 40000\nlocked pulse length = abc\n", {"0631": h(TP5)})
emit("tp-short-31", ["tp"], "", {"0631": h(TP5[:31])})
emit("tp-silent", ["tp"], "", {})
# navigation engine
emit("nav-defaults", ["nav"], "", {"0624": h(NAV5)})
emit("nav-custom", ["nav"], "[navigation engine]\ndynamic model = 3\nfix mode = 3\nfixed altitude (2d) = 1.15\nfixed altitude variance (2d) = 0.0005\nminimum elevation = -5\nposition dop mask = 2.3\ntime dop mask = 99.95\nposition accuracy mask = 100\ntime accuracy mask = 350\nstatic hold threshold = 5\ndynamic gnss timeout = 30\nthreshold above c/no = 0\nc/no threshold = 35\nstatic hold max distance = 7\nutc standard = 6\n", {"0624": h(NAV5)})
emit("nav-negative-scaled", ["nav"], "[navigation engine]\nfixed altitude (2d) = -1.15\nfixed altitude variance (2d) = -0.005\n", {"0624": h(NAV5)})
emit("nav-bad-values", ["nav"], "[navigation engine]\nminimum elevation = 200\nposition dop mask = 2.5x\nfix mode = two\n", {"0624": h(NAV5)})
emit("nav-limits", ["nav"], "[navigation engine]\nfixed altitude (2d) = 21474836.47\nfixed altitude variance (2d) = 429496.7295\nposition dop mask = 6553.5\ntime dop mask = 6553.5\n", {"0624": h(NAV5)})
emit("nav-out-of-range", ["nav"], "[navigation engine]\nfixed altitude (2d) = 30000000\nfixed altitude variance (2d) = -1\nposition dop mask = 7000\ntime dop mask = -0.1\n", {"0624": h(NAV5)})
emit("nav-short-35", ["nav"], "", {"0624": h(NAV5[:35])})
emit("nav-silent", ["nav"], "", {})
# configure for timing
emit("timing-defaults", ["timing"], "", dict(ALL))
emit("timing-custom-nmea", ["timing"], "[nmea]\ngga = on\nzda = off\ngsv = yes\n", {**ALL, "0601": h(MSG((0, 1, 0, 0, 0, 0), 4))})
emit("timing-stateful", ["timing"], "[nmea]\ngga = on\n", dict(ALL), stateful=True)
emit("timing-no-gnss-reply", ["timing"], "", {k: v for k, v in ALL.items() if k != "063E"})
emit("timing-no-tp5-reply", ["timing"], "", {k: v for k, v in ALL.items() if k != "0631"})
emit("timing-no-nav5-reply", ["timing"], "", {k: v for k, v in ALL.items() if k != "0624"})
emit("timing-no-msg-reply", ["timing"], "", {k: v for k, v in ALL.items() if k != "0601"})
emit("timing-no-nmea-reply", ["timing"], "", {k: v for k, v in ALL.items() if k != "0617"})
