import subprocess, sys, tempfile
G, H, which = sys.argv[1], sys.argv[2], sys.argv[3]
TP5 = bytes(32); NAV5 = bytes(36)
key, body = ("0631", TP5) if which == "tp5" else ("0624", NAV5)
def run(ini):
    with tempfile.TemporaryDirectory() as d:
        ip, lp = d + "/x.ini", d + "/log"
        open(ip, "w").write(ini)
        fake = subprocess.Popen([sys.executable, "-I", G + "/fake_any.py", lp, f"{key}={body.hex()}"], stdout=subprocess.PIPE, text=True)
        port = fake.stdout.readline().strip()
        try: out = subprocess.run([H, port, ip, which], capture_output=True, text=True, timeout=30)
        finally: fake.kill(); fake.wait()
        sent = next((l.split()[1] for l in open(lp).read().splitlines() if l.startswith("SET ")), None)
        return out.returncode, out.stderr.strip(), (bytes.fromhex(sent)[6:-2] if sent else None)
def le(b, o, n, signed=False): return int.from_bytes(b[o:o+n], "little", signed=signed)
for a in sys.argv[4:]:
    label, ini = a.split("=", 1); ini = ini.encode().decode("unicode_escape")
    rc, err, b = run(ini)
    if b is None: print(f"[{label}] rc={rc} NO SET SENT  stderr={err}"); continue
    if which == "tp5":
        st = (f"cable={le(b,4,2,True)} rf={le(b,6,2,True)} unlP={le(b,8,4)} lockP={le(b,12,4)} unlL={le(b,16,4)} lockL={le(b,20,4)} "
              f"delay={le(b,24,4,True)}")
    else:
        st = (f"dyn={b[2]} fix={b[3]} minElev={le(b,12,1,True)} pacc={le(b,18,2)} tacc={le(b,20,2)} hold={b[22]} dgnss={b[23]} "
              f"nsv={b[24]} cno={b[25]} holdDist={le(b,28,2)} utc={b[30]}")
    print(f"[{label}] rc={rc}\n  {st}\n  stderr: {err.replace(chr(10), ' || ') or '(none)'}")
