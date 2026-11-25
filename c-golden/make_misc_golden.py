#!/usr/bin/env python3
"""Golden data for CFG-MSG / CFG-PRT / CFG-RATE / NAV-PVT / NAV-SAT: runs the REAL gpsctl functions (misc_harness.c)
against fake_any.py and prints one line per case:  <kind> <name> : <key=bodyhex configured> : <outputs, '|' separated>
usage: make_misc_golden.py <harness> <dir with fake_any.py>   (writes to stdout)"""
import os, subprocess, sys, tempfile
H, G = sys.argv[1], sys.argv[2]

def run(args, bodies, stateful=False):
    with tempfile.TemporaryDirectory() as d:
        lp = d + "/log"
        env = dict(os.environ, FAKE_STATEFUL="1" if stateful else "0")
        fake = subprocess.Popen([sys.executable, "-I", G + "/fake_any.py", lp] + [f"{k}={v}" for k, v in bodies.items()],
                                stdout=subprocess.PIPE, text=True, env=env)
        port = fake.stdout.readline().strip()
        try: out = subprocess.run([H, port] + args, capture_output=True, text=True, timeout=60)
        finally: fake.kill(); fake.wait()
        sets = [l.split()[1].upper() for l in open(lp).read().splitlines() if l.startswith("SET ")]
        res = [l for l in out.stdout.splitlines()] + [l.strip() for l in out.stderr.splitlines() if l.startswith("ERR")]
        return res, sets

def hx(b): return bytes(b).hex().upper()
def le(v, n): return v.to_bytes(n, "little")
def cfg(*parts): return b"".join(parts)

def emit(kind, name, bodies, args, stateful=False, show=None):
    res, sets = run(args, bodies, stateful)
    shown = {show: bodies[show]} if show else bodies
    conf = " ".join(f"{k}={' '.join(v[i:i+2] for i in range(0, len(v), 2)) or '-'}" for k, v in shown.items())
    out = "|".join(res + [f"SENT {' '.join(s[i:i+2] for i in range(0, len(s), 2))}" for s in sets])
    print(f"{kind} {name} : {args[0]} {' '.join(args[1:])} : {conf} : {out}")

# --- CFG-MSG (0601): msgClass, msgID, six rates (I2C, UART1, UART2, USB, SPI, reserved)
def msg(rates, id_=0): return bytes([0xF0, id_] + rates)
emit("msgq", "uart1-off", {"0601": hx(msg([2, 0, 3, 4, 5, 6]))}, ["msgq", "0"])
emit("msgq", "uart1-on", {"0601": hx(msg([0, 1, 0, 0, 0, 0], 4))}, ["msgq", "4"])
emit("msgq", "uart1-rate5", {"0601": hx(msg([0, 5, 0, 0, 0, 0], 3))}, ["msgq", "3"])
emit("msgq", "len4", {"0601": hx(msg([9, 1])[:4])}, ["msgq", "0"])
emit("msgq", "len3-short", {"0601": hx(msg([9, 1])[:3])}, ["msgq", "0"])
emit("msgset", "enable-keeps-other-ports", {"0601": hx(msg([2, 0, 3, 4, 5, 6], 1))}, ["msgset", "1", "1"])
emit("msgset", "disable-keeps-other-ports", {"0601": hx(msg([2, 1, 3, 4, 5, 6], 5))}, ["msgset", "5", "0"])
emit("msgset", "already-enabled-no-send", {"0601": hx(msg([0, 1, 0, 0, 0, 0], 4))}, ["msgset", "4", "1"])
emit("msgset", "rate5-counts-as-enabled", {"0601": hx(msg([0, 5, 0, 0, 0, 0], 4))}, ["msgset", "4", "1"])
emit("msgset", "already-disabled-no-send", {"0601": hx(msg([7, 0, 7, 7, 7, 7], 2))}, ["msgset", "2", "0"])
emit("msgset", "len7-short-no-send", {"0601": hx(msg([2, 0, 3, 4, 5, 6])[:7])}, ["msgset", "0", "1"])
emit("msgset", "len9-longer-echoed", {"0601": hx(msg([2, 0, 3, 4, 5, 6]) + b"\xAA")}, ["msgset", "0", "1"])

# --- CFG-PRT (0600), UART: portID=1, res, txReady, mode, baud@8, inProto@12, outProto@14, flags@16, res@18
def prt(baud, out_proto=0x0003, n=20):
    return (bytes([1, 0]) + le(0, 2) + le(0x8D0, 4) + le(baud, 4) + le(0x0007, 2) + le(out_proto, 2) + le(0, 2) + le(0, 2))[:n]
emit("nmeaon", "nmea+ubx", {"0600": hx(prt(9600, 0x0003))}, ["nmeaon"])
emit("nmeaon", "ubx-only", {"0600": hx(prt(9600, 0x0001))}, ["nmeaon"])
emit("nmeaon", "nmea-only", {"0600": hx(prt(9600, 0x0002))}, ["nmeaon"])
emit("nmeaon", "len16", {"0600": hx(prt(9600, 0x0002, 16))}, ["nmeaon"])
emit("nmeaon", "len15-short", {"0600": hx(prt(9600, 0x0002, 15))}, ["nmeaon"])
emit("baud", "9600-to-115200-takes", {"0600": hx(prt(9600))}, ["baud", "115200"], stateful=True)
emit("baud", "115200-to-38400-takes", {"0600": hx(prt(115200, 0x0007))}, ["baud", "38400"], stateful=True)
emit("baud", "receiver-ignores-change", {"0600": hx(prt(9600))}, ["baud", "115200"], stateful=False)
emit("baud", "already-at-rate", {"0600": hx(prt(115200))}, ["baud", "115200"], stateful=True)
emit("baud", "short-body-19", {"0600": hx(prt(9600, n=19))}, ["baud", "115200"], stateful=True)

# --- CFG-RATE (0608) read through the real ubxGetConfig(); the other packets it polls get plausible fixed bodies
OTHERS = {"0613": "00 00 00 00", "063E": hx(bytes([0, 32, 32, 1, 0, 8, 16, 0, 1, 0, 0, 0])),
          "0624": hx(bytes(36)), "0631": hx(bytes(32)), "0617": hx(bytes(12)), "0686": hx(bytes(8)), "0600": hx(prt(9600)), "0601": hx(msg([0, 1, 0, 0, 0, 0]))}
OTHERS = {k: v.replace(" ", "") for k, v in OTHERS.items()}
for name, body in [("1hz-gps", le(1000, 2) + le(1, 2) + le(1, 2)), ("10hz-utc", le(100, 2) + le(1, 2) + le(0, 2)),
                   ("slow-galileo", le(65535, 2) + le(65535, 2) + le(4, 2)), ("len5-short", le(1000, 2) + le(1, 2) + b"\x01"),
                   ("len8-longer", le(250, 2) + le(2, 2) + le(3, 2) + b"\xEE\xEE")]:
    emit("rate", name, dict(OTHERS, **{"0608": hx(body)}), ["rate"], show="0608")

# --- NAV-PVT (0107), 92 bytes
def pvt(year=2026, month=11, day=19, hour=12, minute=30, sec=15, valid=0x07, tacc=25, nano=-300, fixtype=3, flags=1,
        nsv=11, lon=-1.2345678, lat=51.5, h=123456, hmsl=78901, hacc=2100, vacc=3200, gspeed=1500, head=27000000,
        sacc=120, hdgacc=250000, n=92):
    b = bytearray(92)
    b[0:4] = le(123456000, 4); b[4:6] = le(year, 2); b[6:11] = bytes([month, day, hour, minute, sec]); b[11] = valid
    b[12:16] = le(tacc, 4); b[16:20] = (nano & 0xFFFFFFFF).to_bytes(4, "little"); b[20] = fixtype; b[21] = flags; b[23] = nsv
    b[24:28] = (round(lon * 1e7) & 0xFFFFFFFF).to_bytes(4, "little"); b[28:32] = (round(lat * 1e7) & 0xFFFFFFFF).to_bytes(4, "little")
    b[32:36] = le(h, 4); b[36:40] = le(hmsl, 4); b[40:44] = le(hacc, 4); b[44:48] = le(vacc, 4)
    b[60:64] = le(gspeed, 4); b[64:68] = (head & 0xFFFFFFFF).to_bytes(4, "little"); b[68:72] = le(sacc, 4); b[72:76] = le(hdgacc, 4)
    return bytes(b)[:n]
for name, kw in [("typical", {}), ("no-correction", dict(nano=0)), ("positive-correction", dict(nano=450000000)),
                 ("negative-same-second", dict(nano=-300, sec=15)), ("wrap-second-to-minute", dict(sec=0, nano=-300)),
                 ("wrap-to-hour", dict(minute=0, sec=0, nano=-1)), ("wrap-to-day", dict(hour=0, minute=0, sec=0, nano=-1)),
                 ("wrap-to-day-1", dict(day=2, hour=0, minute=0, sec=0, nano=-1)), ("wrap-to-prev-month", dict(day=1, hour=0, minute=0, sec=0, nano=-1)),
                 ("wrap-to-prev-year", dict(month=1, day=1, hour=0, minute=0, sec=0, nano=-1)),
                 ("wrap-to-leap-feb", dict(month=3, day=1, hour=0, minute=0, sec=0, nano=-1, year=2028)),
                 ("wrap-day-date-invalid", dict(valid=0x06, day=0, month=0, year=0, hour=0, minute=0, sec=0, nano=-1)),
                 ("time-not-valid-no-correction", dict(valid=0x00, hour=0, minute=0, sec=0, nano=-500)),
                 ("southern-western", dict(lat=-33.8688, lon=-151.2093, fixtype=2, flags=0)),
                 ("fix-types", dict(fixtype=5, flags=0)), ("len91-short", dict(n=91)), ("len100-longer", dict())]:
    body = pvt(**kw)
    if name == "len100-longer": body += bytes(8)
    emit("fix", name, {"0107": hx(body)}, ["fix"])

# --- NAV-SAT (0135): iTOW u32, version, numSvs, res2, then 12 bytes per SV
def sv(gnss, id_, cno, elev, az, res, flags):
    return bytes([gnss, id_, cno, elev & 0xFF]) + (az & 0xFFFF).to_bytes(2, "little") + (res & 0xFFFF).to_bytes(2, "little") + le(flags, 4)
def sat(svs, claimed=None, extra=b""):
    return le(1000, 4) + bytes([1, len(svs) if claimed is None else claimed, 0, 0]) + b"".join(svs) + extra
F = lambda q=4, used=1, health=1, diff=0, sm=0, orb=1, eph=1, alm=1, ano=0, anoa=0, sbas=0, rtcm=0, pr=0, cr=0, do=0: (
    q | used << 3 | health << 4 | diff << 6 | sm << 7 | orb << 8 | eph << 11 | alm << 12 | ano << 13 | anoa << 14 |
    sbas << 16 | rtcm << 17 | pr << 20 | cr << 21 | do << 22)
emit("sats", "none", {"0135": hx(sat([]))}, ["sats"])
emit("sats", "three-mixed", {"0135": hx(sat([sv(0, 5, 44, 62, 271, 15, F()), sv(6, 12, 30, -5, 359, -32768 + 7, F(q=7, used=0, health=2, orb=7)),
                                           sv(2, 33, 0, 90, 0, 0, F(q=0, used=0, health=0, orb=0, eph=0, alm=0))]))}, ["sats"])
emit("sats", "every-flag", {"0135": hx(sat([sv(3, 200, 255, 127, 360, 32767, 0xFFFFFFFF)]))}, ["sats"])
emit("sats", "corrections", {"0135": hx(sat([sv(1, 120, 40, 30, 100, 5, F(ano=1, anoa=1, sbas=1, rtcm=1, pr=1, cr=1, do=1, diff=1, sm=1, orb=3))]))}, ["sats"])
emit("sats", "count-exceeds-body", {"0135": hx(sat([sv(0, 1, 20, 10, 10, 1, F()), sv(0, 2, 21, 11, 11, 2, F())], claimed=9))}, ["sats"])
emit("sats", "trailing-partial-sv", {"0135": hx(sat([sv(0, 1, 20, 10, 10, 1, F())], claimed=2, extra=b"\x01\x02\x03"))}, ["sats"])
emit("sats", "len7-short", {"0135": hx(sat([])[:7])}, ["sats"])
emit("sats", "len8-count-zero-claimed3", {"0135": hx(sat([], claimed=3))}, ["sats"])
