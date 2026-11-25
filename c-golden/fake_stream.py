#!/usr/bin/env python3
"""Fake receiver on a pty that, after the FIRST complete-or-not host write, sends a fixed byte stream once.
usage: fake_stream.py <stream hex>   prints the slave path, runs until killed."""
import os, pty, select, sys
stream = bytes.fromhex(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1] != "-" else b""
master, slave = pty.openpty()
print(os.ttyname(slave), flush=True)
sent = False
while True:
    r, _, _ = select.select([master], [], [], 0.1)
    if r:
        os.read(master, 4096)
        if not sent:
            sent = True
            os.write(master, stream)
