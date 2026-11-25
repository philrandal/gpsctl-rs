#!/usr/bin/env python3
"""Runs the real C gpsctl and the Rust gpsctl with the same command line, each against its own fresh fake receiver
(fake_any.py, answering polls with realistic bodies), and compares stdout, exit status and every frame the tool SET.
usage: compare_cli.py <c gpsctl> <rust gpsctl> [name-filter]    (needs /etc/gpsctl.conf = the shipped config, for the
configuration-driven commands; the Rust tool reads the same file)"""
import os, struct, subprocess, sys, time, signal

HERE = os.path.dirname(os.path.abspath(__file__))
C, RUST = sys.argv[1], sys.argv[2]
FILTER = sys.argv[3] if len(sys.argv) > 3 else ""

def u16(x): return struct.pack("<H", x)
def u32(x): return struct.pack("<I", x & 0xFFFFFFFF)
def i32(x): return struct.pack("<i", x)

def padded(s, n): return s.encode().ljust(n, b"\0")
monver = padded("ROM CORE 3.01 (107888)", 30) + padded("00080000", 10) + padded("FWVER=SPG 3.01", 30) + padded("PROTVER=18.00", 30) + padded("GPS;GLO;GAL;BDS", 30)

def pvt(valid=7, fix=3, flags=1, nano=-537, lat=515000000, lon=-12345678, hmsl=45678):
    b = u32(123456) + u16(2026) + bytes([11, 21, 13, 5, 5, valid]) + u32(30) + i32(nano) + bytes([fix, flags, 0, 14])
    b += i32(lon) + i32(lat) + i32(52000) + i32(hmsl) + u32(2500) + u32(3100)
    b += i32(10) + i32(20) + i32(30) + i32(500) + i32(1234567) + u32(100) + u32(250000) + u16(120)
    return (b + bytes(92 - len(b)))

def sat(gnss, sv, cno, el, az, res, flags): return bytes([gnss, sv, cno, el & 255]) + struct.pack("<hh", az, res) + u32(flags)
sats = u32(1) + bytes([1, 5, 0, 0]) + b"".join([
    sat(0, 5, 40, 60, 120, -15, 0x1 | 8 | 0x10 | (4 << 8) | (1 << 11) | (1 << 12)),
    sat(2, 11, 35, 30, 200, 7, 5 | 8 | 0x10 | (1 << 11) | (1 << 16)),
    sat(0, 9, 40, 10, 300, 3, 4 | 0x10 | (2 << 8) | (1 << 12)),
    sat(6, 3, 20, -5, 359, 3276, 2 | 0x20 | (5 << 8)),
    sat(3, 22, 28, 45, 90, 0, 7 | 8 | 0x10 | (1 << 11) | (1 << 6) | (1 << 7) | (1 << 17) | (1 << 20) | (1 << 21) | (1 << 22) | (1 << 13) | (1 << 14)),
])

gnss = bytes.fromhex("00202007" "00081000 01000101" "01010300 00000101" "02040800 01000101" "03081000 00000101" "04000800 00000103" "05000300 00000105" "06080e00 00000101")
nav5 = bytes.fromhex("ffff0202" "00000000" "01000000" "1400" "6400" "6400" "2800" "2800" "003c0814" "0000" "0000" "03" "0000000000")  # 36 bytes
tp5 = bytes.fromhex("0001000038001400" "40420f00" "40420f00" "00000000" "20a10700" "00000000" "77000000")
prt = bytes.fromhex("0100" "0000" "d0080000" "00c20100" "0700" "0300" "0000" "0000")
nmea = bytes.fromhex("00410000" "00000000" "00000100" "00000000" "00000000")
KEYS = {
    "0a04": monver, "0107": pvt(), "0135": sats, "0613": bytes.fromhex("1f000000"), "063e": gnss, "0624": nav5, "0631": tp5,
    "0608": bytes.fromhex("e80301000100"), "0686": bytes.fromhex("00010a0005000000"), "0600": prt, "0617": nmea,
    "0601": bytes.fromhex("f0000001000101" "00"), "0609": b"\0", "0604": b"\0",
}

def run(tool, args, env_extra, keys, stateful, timeout=60):
    log = f"/tmp/cmp_{os.getpid()}_{os.path.basename(tool)}.log"
    fake = subprocess.Popen([sys.executable, os.path.join(HERE, env_extra.get("FAKE_SCRIPT", "fake_any.py")), log] + [f"{k}={v.hex()}" for k, v in keys.items()],
                            stdout=subprocess.PIPE, text=True, env={**os.environ, **env_extra, "FAKE_STATEFUL": "1" if stateful else "0"})
    path = fake.stdout.readline().strip()
    try:
        t0 = time.time()
        r = subprocess.run([tool] + [path if a == "@PORT" else a for a in args], capture_output=True, text=True,
                           timeout=timeout, env={**os.environ, **env_extra})
        dt = time.time() - t0
        time.sleep(0.15)
        sets = open(log).read().replace(path, "@PORT") if os.path.exists(log) else ""
        return r.returncode, r.stdout.replace(path, "@PORT"), r.stderr.replace(path, "@PORT"), sets, dt
    finally:
        fake.kill(); fake.wait()

P = ["-p", "@PORT", "--shared_port"] + (os.environ.get("EXTRA_ARGS", "").split() or [])
scen = []
def S(name, args, keys=None, stateful=False, env=None): scen.append((name, args, keys or KEYS, stateful, env or {}))
for q in ["fix", "version", "satellites", "config"]:
    S(f"query {q}", P + ["-Q", q]); S(f"query {q} json", P + ["-Q", q, "-j"]); S(f"query {q} -vv", P + ["-Q", q, "-vv"])
S("fix not valid", P + ["-Q", "fix"], {**KEYS, "0107": pvt(valid=0, fix=0, flags=0, nano=0, lat=0, lon=0)})
S("fix 2d south west", P + ["-Q", "fix"], {**KEYS, "0107": pvt(fix=2, lat=-335000000, lon=-700000000, hmsl=-5000)})
S("fix 2d south west json", P + ["-Q", "fix", "-j"], {**KEYS, "0107": pvt(fix=2, lat=-335000000, lon=-700000000, hmsl=-5000)})
S("fix midnight borrow", P + ["-Q", "fix"], {**KEYS, "0107": pvt(nano=-900000000)})
S("no sats", P + ["-Q", "satellites"], {**KEYS, "0135": u32(1) + bytes([1, 0, 0, 0])})
S("short sat body", P + ["-Q", "satellites"], {**KEYS, "0135": b"\x01\x02\x03"})
S("config short anten", P + ["-Q", "config"], {**KEYS, "0613": b"\x01"})
S("config power/interval", P + ["-Q", "config"], {**KEYS, "0686": bytes.fromhex("0002 0a00 0500 0000".replace(" ", "")), "0631": tp5[:12] + bytes.fromhex("0a000000") + tp5[16:28] + bytes.fromhex("7f000000")})
S("config freq mode", P + ["-Q", "config", "-j"], {**KEYS, "0631": tp5[:28] + bytes.fromhex("5f000000")})
S("config freq mode text", P + ["-Q", "config"], {**KEYS, "0631": tp5[:28] + bytes.fromhex("5f000000")})
S("nmea yes", P + ["-n", "yes"]); S("nmea no", P + ["-n", "no", "-v"], stateful=True)
S("nmea on + version", P + ["-n", "on", "-Q", "version"], stateful=True)
S("nmea off already", P + ["-n", "n", "-Q", "version", "-vv"], {**KEYS, "0600": prt[:14] + b"\x01\x00" + prt[16:]})
S("save", P + ["--save_config"]); S("reset", P + ["--reset"]); S("satellites", P + ["--satellites"], stateful=True)
S("galileo", P + ["--galileo"], stateful=True);
gnss_glo = gnss[:-8] + bytes.fromhex("06080e00 00000100")           # receiver has GLONASS off, the config says on
gnss_noGal = gnss[:4 + 8 * 2] + bytes.fromhex("02040800 00000101") + gnss[4 + 8 * 3:]  # receiver has Galileo off
gnss_same = gnss[:4+8*2]+bytes.fromhex("02040800 01000101")+gnss[4+8*3:]
S("sats unchanged", P + ["--satellites"], {**KEYS, "063e": gnss_same}, stateful=True)
S("sats glonass differs", P + ["--satellites"], {**KEYS, "063e": gnss_glo}, stateful=True)
S("sats galileo differs", P + ["--satellites"], {**KEYS, "063e": gnss_noGal}, stateful=True)
S("timing galileo differs", P + ["--configure_for_timing"], {**KEYS, "063e": gnss_noGal}, stateful=True)
S("sats galileo differs, silent after reset", P + ["--satellites"], {**KEYS, "063e": gnss_noGal}, stateful=True, env={"FAKE_SILENT_AFTER_RST": "1"})
S("sats galileo differs, save silent", P + ["--satellites"], {**KEYS, "063e": gnss_noGal, "0609": b"-"}, stateful=True)
S("sats short gnss", P + ["--satellites"], {**KEYS, "063e": gnss[:8]}, stateful=True)
S("timing", P + ["--configure_for_timing"], stateful=True)
S("order", P + ["--save_config", "-Q", "version", "--satellites", "-n", "yes"], stateful=True)
S("save silent", P + ["--save_config"], {**KEYS, "0609": b"-"})
FR = {**KEYS, "0609": bytes(13), "0a04": monver}
FB = ["-b", "115200"]
S("factory reset", P + FB + ["--factory_reset=yes"], FR, env={"FAKE_BAUD": "115200"})
S("factory reset then query", P + FB + ["--factory_reset=yes", "-Q", "version"], FR, env={"FAKE_BAUD": "115200"})
S("factory reset then nmea", P + FB + ["--factory_reset=yes", "-n", "no"], FR, stateful=True, env={"FAKE_BAUD": "115200"})
S("factory reset arg no", P + FB + ["--factory_reset=no"], FR)
S("factory reset arg YES", P + FB + ["--factory_reset=YES"], FR)
S("factory reset no arg", P + FB + ["--factory_reset"], FR)
S("factory reset space arg", P + FB + ["--factory_reset", "yes"], FR, env={"FAKE_BAUD": "115200"})
S("factory reset stays at old baud", P + FB + ["--factory_reset=yes"], FR, env={"FAKE_BAUD": "115200", "FAKE_NOSWITCH": "1"})
S("factory reset silent after restart", P + FB + ["--factory_reset=yes"], FR, env={"FAKE_BAUD": "115200", "FAKE_SILENT_AFTER_RST": "1"})
S("reset silent", P + ["--reset"], {**KEYS, "0604": b"-"})
S("timing silent tp5", P + ["--configure_for_timing"], {**KEYS, "0631": b"-"}, stateful=True)
S("timing silent nav5", P + ["--configure_for_timing"], {**KEYS, "0624": b"-"}, stateful=True)
S("satellites silent", P + ["--satellites"], {**KEYS, "063e": b"-"}, stateful=True)
S("nmea silent prt", P + ["-n", "no"], {**KEYS, "0600": b"-"})
S("sync ubx", P + ["-s"]); S("sync u", P + ["-s", "u"]); S("sync bad", P + ["-s", "x"]); S("sync=ubx", P + ["--sync=ubx", "-vv"])
S("baud 9600 query", P + ["-b", "9600", "-Q", "version", "-vv"])
S("newbaud noop", P + ["-B", "115200", "-vv"]); S("newbaud 230400", P + ["-B", "230400", "-vv"], stateful=True)
S("autobaud ubx", P + ["-a", "-vv"]); S("autobaud nmea silent", P + ["-a", "nmea", "-M", "115200"])
S("autobaud confirmed", P + ["-a", "-v"], {**KEYS, "0600": prt[:8] + u32(921600) + prt[12:]})
S("autobaud confirmed -vv + nmea off", P + ["-a", "-vv", "-n", "no"], {**KEYS, "0600": prt[:8] + u32(921600) + prt[12:]}, stateful=True)
S("autobaud confirmed quiet", P + ["-q", "-a"], {**KEYS, "0600": prt[:8] + u32(921600) + prt[12:]})
S("autobaud unconfirmable", P + ["-a", "-vv"], {**KEYS, "0600": b"-"})
S("echo 1", P + ["-e1"]); S("echo=1 vv", P + ["--echo=1", "-vv"])
S("quiet query", P + ["-q", "-Q", "version"])
S("silent receiver", P + ["-Q", "version"], {**KEYS, "0a04": b"-"})
S("silent receiver vv", P + ["-Q", "version", "-vv"], {**KEYS, "0a04": b"-"})
S("silent fix", P + ["-Q", "fix"], {**KEYS, "0107": b"-"}); S("silent version", P + ["-Q", "version"], {**KEYS, "0a04": b"-"}); S("silent sats", P + ["-Q", "satellites"], {**KEYS, "0135": b"-"})
S("silent cfg", P + ["-Q", "config", "-vv"], {**KEYS, "0613": b"-"})
S("bad baud", P + ["-b", "1234"]); S("bad baud text", P + ["-b", "abc"]); S("big baud", P + ["-b", "99999999999"])
S("bad query", P + ["-Q", "bogus"]); S("unknown short", P + ["-x"]); S("unknown long", P + ["--bogus"])
S("sync+autobaud", P + ["-s", "-a"]); S("json no query", P + ["-j"]); S("minbaud no auto", P + ["-M", "9600"])
S("quiet+verbose", P + ["-q", "-v"]); S("missing arg", P + ["-b"]); S("missing arg long", P + ["--port"])
S("too many v", P + ["-vvv"]); S("long with arg on flag", P + ["--quiet=5"]); S("long junk suffix", P + ["--portx"])
S("nmea missing", P + ["-n"]); S("echo not int", P + ["-eabc"]); S("query attached", P + ["-Qversion"]); S("cluster", P + ["-vjQ", "version"])
S("repeat option", P + ["-b", "9600", "-b", "9600"]); S("repeat json", P + ["-Q", "fix", "-j", "-j", "-j", "-j"])
S("double dash", P + ["--", "-Q", "version"]); S("non-option args", P + ["foo", "-Q", "version", "bar"])
S("extra word", P + ["foo"]); S("echo with space", P + ["-e", "1"]); S("words after dash dash", P + ["foo", "--", "bar"])
S("extra word with query", P + ["-Q", "version", "foo"], ); S("extra word vv", P + ["-vv", "-Q", "version", "foo"])
S("extra word and conflict", P + ["foo", "-q", "-v"]); S("version with extra", P + ["-V", "foo"]); S("help with extra", P + ["--help", "foo"])
S("negative echo", P + ["--echo=-1"]); S("negative echo vv", P + ["-vv", "--echo=-1"]); S("negative echo short", P + ["-e=-5"])
S("not a terminal", ["-p", "/dev/null"]); S("no such port", ["-p", "/dev/nonexistent"])

fails = 0; n = 0
for name, args, keys, stateful, env in scen:
    if FILTER and FILTER not in name: continue
    n += 1
    a = run(C, args, env, keys, stateful)
    b = run(RUST, args, env, keys, stateful)
    same = a[:4] == b[:4]
    print(("ok   " if same else "DIFF ") + name + f"   (exit {a[0]}/{b[0]}, {a[4]:.1f}s/{b[4]:.1f}s)", flush=True)
    if not same:
        fails += 1
        for label, i in (("stdout", 1), ("sets", 3)):
            if a[i] != b[i]:
                print(f"  --- C {label}:\n{a[i]}  --- Rust {label}:\n{b[i]}")
        if a[0] != b[0]: print(f"  exit: C {a[0]}, Rust {b[0]}")
        if a[2] != b[2]: print(f"  stderr: C {a[2]!r}, Rust {b[2]!r}")
print(f"\n{n - fails} of {n} identical")
sys.exit(1 if fails else 0)
