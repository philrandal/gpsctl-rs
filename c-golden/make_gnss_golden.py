#!/usr/bin/env python3
"""Runs the REAL ubxConfigSatellites() (gnss_harness) against fake_gnss.py for each case below and writes golden_gnss.txt.
usage: make_gnss_golden.py <gnss_harness binary> > golden_gnss.txt      (receiver bodies are plausible M8-style SAMPLES, not captured)"""
import os, subprocess, sys, tempfile, time

HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = sys.argv[1]

def blocks(*bs):  # header: msgVer 0, numTrkChHw 22, numTrkChUse 22, numConfigBlocks, then 8-byte blocks
    return bytes([0, 22, 22, len(bs)]) + b"".join(bs)

GPS     = bytes.fromhex("0008100001000101")
SBAS    = bytes.fromhex("0101030001000101")
GAL     = bytes.fromhex("0204080000000101")
BDS     = bytes.fromhex("0308100000000101")
IMES    = bytes.fromhex("0400080000000103")
QZSS    = bytes.fromhex("0500030001000505")
GLO     = bytes.fromhex("06080E0001000101")
UNKNOWN = bytes.fromhex("0902050001000101")   # a GNSS id gpsctl doesn't know: must pass through untouched
ALL7 = blocks(GPS, SBAS, GAL, BDS, IMES, QZSS, GLO)
NMEA20 = bytes.fromhex("0041000200000000000000010000000000000000")   # real captured M8 CFG-NMEA body

cases = [
  ("defaults",            "", ALL7),
  ("beidou-loses-to-glonass", "[beidou]\nenabled = yes\n", ALL7),
  ("prefer-beidou",       "[beidou]\nenabled = yes\n[gpsctl]\nprefer beidou to glonass if both enabled = yes\n", ALL7),
  ("gps-only-nmea23",     "[galileo]\nenabled = no\n[glonass]\nenabled = no\n[nmea]\nversion = 2.3\n", ALL7),
  ("beidou-and-glonass-no-gps-gal", "[gps]\nenabled = no\n[galileo]\nenabled = no\n[beidou]\nenabled = yes\n[nmea]\nversion = 4.0\n", ALL7),
  ("custom-channels-sbas-on", "[gps]\nminimum channels = 10\nmaximum channels = 20\n[sbas]\nenabled = yes\nmaximum channels = 2\n[qzss]\nenabled = true\n", ALL7),
  ("bool-forms",          "[gps]\nenabled = N\n[galileo]\nenabled = T\n[glonass]\nenabled = 0\n[sbas]\nenabled = Yes\n", ALL7),
  ("bool-off-is-invalid", "[galileo]\nenabled = off\n[glonass]\nenabled = disabled\n", ALL7),
  ("unknown-block",       "", blocks(GPS, UNKNOWN, GLO)),
  ("count-larger-than-body", "", bytes([0, 22, 22, 255]) + GPS + GAL),
  ("count-smaller-than-body", "", bytes([0, 22, 22, 1]) + GPS + GAL),
  ("short-body",          "", bytes([0, 22, 22, 1, 0, 8, 16, 0])),
  ("chan300-rejected",  "[gps]\nmaximum channels = 300\n", ALL7),
  ("nmea-already-41",     "[nmea]\nversion = 41\n", ALL7),
]

for name, ini, body in cases:
    with tempfile.TemporaryDirectory() as d:
        inip, logp = os.path.join(d, "x.ini"), os.path.join(d, "log")
        open(inip, "w").write(ini)
        fake = subprocess.Popen([sys.executable, "-I", os.path.join(HERE, "fake_gnss.py"), body.hex(), NMEA20.hex(), logp],
                                stdout=subprocess.PIPE, text=True)
        port = fake.stdout.readline().strip()
        try:
            out = subprocess.run([HARNESS, port, inip], capture_output=True, text=True, timeout=30)
        finally:
            fake.kill(); fake.wait()
        log = open(logp).read().splitlines()
        if out.returncode != 0 or "AddressSanitizer" in out.stderr:
            sys.exit(f"{name}: harness failed rc={out.returncode}\n{out.stderr}")
        gnss = next((l.split()[1] for l in log if l.startswith("GNSS ")), "none")
        nmea = next((l.split()[1] for l in log if l.startswith("NMEA ")), "none")
        result = out.stdout.strip().splitlines()[-1]
        print("\t".join([name, ini.replace("\n", "\\n"), body.hex(), NMEA20.hex(), gnss, nmea, result]))
