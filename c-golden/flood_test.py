#!/usr/bin/env python3
"""usage: flood_test.py <tool> [args...]  runs the tool against fake_flood.py (receiver at 9600, dense NMEA) and prints the
exit status, the time taken, the receiver's poll log (good / CORRUPT replies) and the tool's output."""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
tool, args = sys.argv[1], sys.argv[2:]
sys.argv = ["x", "/bin/true", "/bin/true", "NOMATCH"]
exec(open(os.path.join(HERE, "compare_cli.py")).read().split("scen = []")[0])
prt = bytearray(KEYS["0600"]); prt[8:12] = (9600).to_bytes(4, "little"); KEYS["0600"] = bytes(prt)
r = run(tool, ["-p", "@PORT", "--shared_port"] + args, {"FAKE_SCRIPT": "fake_flood.py", "FAKE_BAUD": "9600"}, KEYS, False, timeout=120)
log = open(f"/tmp/cmp_{os.getpid()}_{os.path.basename(tool)}.log").read()
print("exit", r[0], "time %.1fs" % r[4]); print("--- receiver log:\n" + log + "--- stdout:\n" + r[1] + "--- stderr:\n" + r[2])
