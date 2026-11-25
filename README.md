# gpsctl-rs

A Rust port of [gpsctl](https://github.com/philrandal/gpsctl), the u-blox GPS control tool for the Raspberry Pi.
It has the same options, configuration file, output, JSON and exit status as the C program (release 1.42), and
uses only the Rust standard library (no crates).

Tested so far: the 75 unit tests, a comparison against the real C program in 101 scenarios (identical output
and identical frames sent to a fake receiver), and the main options on a real u-blox M8 (protocol 18) on a
Raspberry Pi UART. See [docs/STATUS.md](docs/STATUS.md) for what has and has not been verified.

## Building

```
./build.sh                 # builds dist/gpsctl-aarch64-unknown-linux-gnu (a 64-bit Raspberry Pi binary)
RUN_TESTS=1 ./build.sh     # runs the test suite on the build machine first
TARGET=x86_64-unknown-linux-gnu ./build.sh    # another target, e.g. for testing on a PC
```

On a 64-bit Raspberry Pi (aarch64) this is a plain native build. On any other machine it cross-compiles to
aarch64. Copy the resulting file to the Pi and run it as `gpsctl` (`gpsctl --help` lists the options).

### Prerequisites (Debian, Ubuntu, Raspberry Pi OS and other Debian variants)

All variants: a C linker driver and the Rust toolchain. Use `rustup` rather than Debian's `rustc` package,
which can be too old (the code is edition 2021 and was built with Rust 1.97; the minimum version has not been
determined).

```
sudo apt update
sudo apt install build-essential curl ca-certificates file
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
. "$HOME/.cargo/env"
```

**Building on the Raspberry Pi itself** (64-bit Raspberry Pi OS, `uname -m` shows `aarch64`): the above is all
you need, then `./build.sh`. A 32-bit Raspberry Pi OS is not supported by `build.sh`'s default; use
`TARGET=armv7-unknown-linux-gnueabihf ./build.sh` on the Pi (not tried).

**Cross-compiling from an x86-64 Debian/Ubuntu machine** additionally needs the aarch64 cross linker; the
script adds the Rust target (`rustup target add aarch64-unknown-linux-gnu`) for you:

```
sudo apt install gcc-aarch64-linux-gnu
```

`.cargo/config.toml` already tells Cargo to link with `aarch64-linux-gnu-gcc`. The binary is dynamically linked
against glibc, so build on a Debian release no newer than the one on the Pi (or build on the Pi).

The cross-compile path has not been run yet; the native build and the tests have (on x86-64 Linux).

### Optional, for the test tooling in `c-golden/`

`python3` and `gcc` (both in `build-essential`/`python3`), plus a checkout of the C
[gpsctl](https://github.com/philrandal/gpsctl) repository: the golden-data generators compile its real
`ublox.c`, and `c-golden/compare_cli.py <C gpsctl> <Rust gpsctl>` runs both programs against a fake receiver and
compares them. `cargo test` itself needs none of this (the golden files are included in the repository).

## Running

- Needs `stty` (coreutils, always present on Debian) to set the serial line.
- The user must be allowed to open the port: `sudo usermod -aG dialout $USER` (then log in again), or use `sudo`.
- Stop anything else using the port first (`sudo systemctl stop gpsd gpsd.socket`); like the C program, gpsctl
  refuses a port that another process holds (`--shared_port` overrides it).
- Configuration is read from `/etc/gpsctl.conf` (override with `GPSCTL_CONF=/path`); the format is the same as the
  C program's (see its `etc/` directory). `GPSCTL_TRACE=1` prints every byte sent and received.
- Example: `./gpsctl -p /dev/ttyAMA0 -a -vv -Q fix`

## Layout

| path | what |
|---|---|
| `src/` | the library: UBX/NMEA messages, serial layer, synchronizers, commands, config, CLI (`cli.rs`, `queries.rs`) |
| `src/bin/gpsctl.rs` | the `gpsctl` program |
| `examples/nmea_tool.rs` | older diagnostic tool (`cargo run --example nmea_tool -- --help`) |
| `c-golden/` | golden data produced by the real C code, and the C-versus-Rust comparison scripts |
| `tools/fake_m8.py` | a fake receiver on a pty, for trying the tool without hardware |
| `docs/STATUS.md` | status, verification and findings |

Licence: MIT (see `LICENSE`).
