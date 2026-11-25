#!/bin/bash
# Builds the gpsctl binary for a 64-bit Raspberry Pi (aarch64) into dist/.
#
#   ./build.sh                 build dist/gpsctl-aarch64-unknown-linux-gnu
#   RUN_TESTS=1 ./build.sh     run the test suite (on this machine) first
#   TARGET=<triple> ./build.sh build for another Rust target (e.g. x86_64-unknown-linux-gnu)
#
# On an aarch64 machine (a 64-bit Raspberry Pi) this is a plain native build. On any other
# machine it cross-compiles; the prerequisites are listed in README.md.
set -euo pipefail
cd "$(dirname "$0")"

TARGET="${TARGET:-aarch64-unknown-linux-gnu}"
BIN=gpsctl

die() { echo "build.sh: $*" >&2; exit 1; }

command -v cargo >/dev/null 2>&1 || die "cargo not found. Install Rust with rustup (see README.md, 'Prerequisites')."
HOST="$(rustc -vV | sed -n 's/^host: //p')"

if [ "$HOST" != "$TARGET" ]; then
    # cross build: the standard library for the target, and a linker for it, must be installed
    if command -v rustup >/dev/null 2>&1; then
        if ! rustup target list --installed | grep -qx "$TARGET"; then
            echo "Installing the Rust standard library for $TARGET ..."
            rustup target add "$TARGET"
        fi
    else
        [ -d "$(rustc --print sysroot)/lib/rustlib/$TARGET" ] ||
            die "the Rust standard library for $TARGET is not installed (install rustup, see README.md)."
    fi
    if [ "$TARGET" = "aarch64-unknown-linux-gnu" ]; then
        command -v aarch64-linux-gnu-gcc >/dev/null 2>&1 ||
            die "aarch64-linux-gnu-gcc not found. On Debian/Ubuntu: sudo apt install gcc-aarch64-linux-gnu"
    fi
fi

if [ "${RUN_TESTS:-0}" = "1" ]; then
    echo "Running the tests on $HOST ..."
    cargo test --release --locked
fi

echo "Building $BIN for $TARGET ..."
cargo build --release --locked --bin "$BIN" --target "$TARGET"

mkdir -p dist
OUT="dist/$BIN-$TARGET"
cp "target/$TARGET/release/$BIN" "$OUT"
echo "Built $OUT"
file "$OUT" 2>/dev/null || true
echo "Copy it to the Pi, e.g.:  scp $OUT pi@raspberrypi:gpsctl"
