<h1 align="center"><b>gpsctl-rs 2.00</b></h1>

A Rust port of [gpsctl](https://github.com/philrandal/gpsctl), a utility for configuring and querying u-blox GPS
receivers from a Raspberry Pi (or any Linux host) over a serial port. It has the same options, configuration file,
output, JSON and exit status as release 2.00 of the C program, and uses only the Rust standard library (no crates).

## What is gpsctl?
*gpsctl* is a utility for Raspberry Pi computers using a u-blox GPS board. The original author's rig was a
[Raspberry Pi 3 Model B](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/) with a
[Uputronics GPS board](https://store.uputronics.com/index.php?route=product/product&product*id=81), which uses a
[u-blox MAX-M8Q](https://www.u-blox.com/en/product/max-m8-series) module. Some functions work with any GPS that
streams NMEA data to a Linux serial port; most are specific to u-blox receivers and use their UBX protocol.

Configuring a u-blox GPS from Linux is a frustrating exercise, and support for the UBX protocol is minimal. *gpsctl*
was written to make a [stratum 1 NTP server](https://www.endruntechnologies.com/stratum1.htm) practical: it sets
the receiver up for accurate timing (stationary mode, the satellite systems you want, ZDA output, time pulse
settings) and saves the result.

## Which receivers?
gpsctl is written and tested for the u-blox 8 (M8) generation, which reports a UBX protocol version up to 23 (the
author's module reports 18). Newer generations (u-blox 9 and later, protocol 27 and up) are configured with a different,
key-based interface that gpsctl does not use, so some of its settings may not apply to them. If a newer receiver is
detected, gpsctl says so on stderr before it queries the version or changes the satellites or timing configuration, and
carries on. The restriction on using BeiDou and GLONASS together (see "Configuration file") is a limit of the M8, not of
newer receivers; gpsctl does not yet know how to lift it for them.

## What does it do?
Functions that work for any GPS streaming NMEA over a serial port:
* Echoes NMEA data to stdout.
* Infers the GPS's baud rate from the stream of NMEA data.

Functions specific to u-blox GPS hardware:
* Configures the baud rate that the receiver uses (and the host's).
* Configures whether the receiver transmits NMEA data, and which NMEA sentences and version.
* Infers the baud rate using the UBX protocol. It first listens, sending nothing, for the receiver's own NMEA (a
  u-blox that sees more than 100 frame errors switches its UART receiver off for minutes, and polls sent at a wrong
  baud rate cause them), and polls only at the rate where NMEA is heard.
* Configures the satellite systems (GPS, SBAS, Galileo, BeiDou, GLONASS, QZSS, IMES). After a change with Galileo
  enabled it saves the configuration and restarts the receiver, as u-blox requires.
* Queries position, time, version, configuration and satellite information, as plain English or JSON.
* Configures the receiver for maximum timing accuracy (`--configure_for_timing`): satellites, navigation engine,
  time pulse, NMEA sentences, all from the configuration file.
* Saves the configuration to battery-backed RAM, and to flash, EEPROM and SPI flash if the module has them.
* Resets the receiver, or restores its factory defaults (`--factory_reset=yes`).

Not included: the C program's old PPS-pin test (`--test`), which relied on the unsupported wiringPi library and no
longer exists in the C program either. To check that a PPS signal is arriving, use `ppstest /dev/pps0` or look at
the sources in `chronyc sources` / `ntpq -p`.

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
- Stop anything else using the port first (`sudo systemctl stop gpsd gpsd.socket`). A running gpsd answers the
  autobaud polls itself and gives a false result, so, like the C program, gpsctl refuses a port that another
  process holds (`--shared_port` overrides this).
- Configuration is read from `/etc/gpsctl.conf` (see "Configuration file" below); the environment variable
  `GPSCTL_CONF=/path` names a different file.
- `GPSCTL_TRACE=1` prints every byte sent to and received from the serial port, with times, to stderr (for finding
  out what really happens on the port).
- Run it by explicit path (`./gpsctl`) until you have installed it, in case another copy is earlier in `PATH`.
- Install: `sudo install -m 755 dist/gpsctl-aarch64-unknown-linux-gnu /usr/local/bin/gpsctl`.

### Start-up at boot
`etc/systemd/system/ublox-init.service` runs gpsctl (up to three times, 15 seconds apart) before chrony and gpsd start,
with `PARAMS` taken from `/etc/default/gpsctl` (an example is in `etc/default/gpsctl`).

## A stratum 1 server with chrony and gpsd
`etc/` holds a working set of files for a Raspberry Pi stratum 1 NTP server: a u-blox M8 on `/dev/ttyAMA0` with its
PPS signal on `/dev/pps0`, gpsd reading both, and chrony as the time daemon.  They have been run on a Raspberry Pi 3B
with Raspberry Pi OS (NetworkManager); other setups may need adjusting.

| file | install to | what it does |
|---|---|---|
| `etc/systemd/system/ublox-init.service` | `/etc/systemd/system/` | runs gpsctl at boot, before chrony and gpsd, with options from `/etc/default/gpsctl` |
| `etc/systemd/system/chrony.service.d/wait-for-network.conf` | `/etc/systemd/system/chrony.service.d/` | starts chrony after `network-online.target` |
| `etc/systemd/system/gpsd.service.d/override.conf` | `/etc/systemd/system/gpsd.service.d/` | starts gpsd after chrony |
| `etc/systemd/system/ntp-hw-tune.service` | `/etc/systemd/system/` | sets the CPU governor to `performance` and turns EEE off on `eth0` (ignored where EEE is unsupported, e.g. the Pi 3B) |
| `etc/chrony/chrony.conf` | `/etc/chrony/chrony.conf` | chrony configuration for the GPS and PPS sources |

Boot order is `ublox-init`, then `chrony`, then `gpsd`.  gpsd must start after chrony because it connects to the SOCK
refclock sockets that chrony creates.  `After=network-online.target` only waits if a wait-online service is
enabled (`NetworkManager-wait-online` on Raspberry Pi OS, or `systemd-networkd-wait-online`).

To install:

    sudo cp etc/systemd/system/*.service /etc/systemd/system/
    sudo cp -r etc/systemd/system/*.service.d /etc/systemd/system/
    sudo cp etc/chrony/chrony.conf /etc/chrony/chrony.conf
    sudo systemctl daemon-reload
    sudo systemctl enable ublox-init ntp-hw-tune
    sudo reboot

`ntp-hw-tune` needs `ethtool` installed.  `gpsd` itself is configured in `/etc/default/gpsd`; the author runs it with
`--nowait --badtime --readonly --speed 115200 /dev/ttyAMA0 /dev/pps0`.

### How `chrony.conf` is arranged
* Two gpsd refclocks: the NMEA time over `/var/run/chrony.clk.ttyAMA0.sock` (`refid GPS`, `noselect`) only gives the
  second; the PPS edge over `/var/run/chrony.pps0.sock` (`refid PPS`, `lock GPS`, `prefer`) gives the precision.
* Network servers stay in the configuration as a cross-check.  `trust` is deliberately not set on the PPS source:
  with it, chrony excludes the network servers (they show as `^?`) instead of just not combining them (`^-`).
* `local stratum 10` keeps the server answering if neither GPS nor network is available.
* `leapseclist /usr/share/zoneinfo/leap-seconds.list` supplies leap seconds.  `leapsectz right/UTC` is not used
  because chrony rejected that zone on Raspberry Pi OS ("failed leap second check").  The list comes with the
  `tzdata` package, so normal upgrades keep it current.
* `hwtimestamp`, `lock_all` and `sched_priority 10` reduce timing jitter on a lightly loaded machine.

A healthy `chronyc sources` shows the PPS source selected (`#*`) with an offset of a few hundred nanoseconds or
less, and the network servers as `^-`.

## Usage

    Options for gpsctl:

    -V, --version               display the version of this program
    -U, --usage                 display a short description of this program's usage
    -?, --help                  display this help message
    -v, --verbose               get verbose messages (may repeat up to 2 times)
    -q, --quiet                 suppress normal output
    -p, --port <device>         specify the port device (default is '/dev/serial0')
    -j, --json                  select JSON query output (may repeat up to 3 times)
        --shared_port           allow use of a serial port that another process has open
    -a, --autobaud[=method]     infer baud rate using ascii, nmea, ubx data (default is ubx)
    -b, --baud <baud rate>      specify different host baud rate; any standard rate is allowed
    -M, --minbaud <baud rate>   specify minimum baud rate for autobaud (default is '9600', or "autobaud minimum" in gpsctl.conf); any standard rate is allowed
    -X, --maxbaud <baud rate>   specify maximum baud rate for autobaud (default is '921600', or "autobaud maximum" in gpsctl.conf); any standard rate is allowed
    -s, --sync[=method]         synchronize using ascii, nmea, ubx data (default is ubx)
    -B, --newbaud <baud rate>   change the U-Blox and host baud rates to the specified standard rate (with -a, that rate is also tried first)
    -n, --nmea <yes/no>         configure whether the GPS sends NMEA data to the serial port (see "Yes or no values" below)
    -Q, --query <query type>    query the GPS for info (see "Query types" below)
        --galileo               configure the GPS for Galileo Satellites
        --satellites            configure the GPS Satellites only
        --configure_for_timing  configure the GPS for maximum timing precision
        --save_config           save the GPS configuration to battery-backed RAM, and to flash, EEPROM and SPI flash if the module has them
        --reset                 reset the GPS
        --factory_reset=yes     restore the GPS to its factory defaults, erasing its saved configuration; the GPS goes back to 9600 baud
    -e, --echo[=seconds]        echo human-readable GPS data to stdout for the given time (0 or unspecified means forever)

    Arguments in <angle brackets> are required; they MUST be supplied.
    Arguments in [square brackets] are optional; they may be omitted.

    Additional information:

    Query types:
      fix         returns position, altitude, and time information
      version     returns hardware and software version of the GPS
      config      returns GPS configuration information
      satellites  returns currently visible satellite information from the GPS

    Yes or no values:
      yes         indicated by an initial character of 'y', 'Y', 't', 'T', or '1', or by 'on'
      no          indicated by anything else

    Examples:
      gpsctl --port=/dev/serial1 --baud 9600 --echo=5
        Selects the serial port "/dev/serial1", sets the host serial port baud rate to 9600, and
        echoes any received characters (presumably NMEA data) to stdout.
      gpsctl -p /dev/serial1 -b 9600 -e=5
        Exactly the same as the preceding example, using short options.

    Environment:
      GPSCTL_TRACE=1  prints every byte sent to and received from the serial port, with times, to stderr
                      (for finding out what really happens on the port; unset or 0 turns it off)

If a number of seconds is given to `-e` it must be attached (`-e5`, `--echo=5`); with a space (`-e 5`) the echo
runs forever and `5` is treated as a stray argument.

### Autobaud
With `-a`, gpsctl works out the receiver's baud rate by trying the standard rates from the maximum down to the
minimum (921600 down to 9600 unless told otherwise).  Each rate costs about 1.5 seconds when the receiver is sending
NMEA, and more when it is not, so narrowing the range or saying where to look first saves time at every start-up:
* `-B <rate>` together with `-a`: that rate is tried first, so a receiver that is already there is found at once.
  The usual boot-time `-a -B 115200` finds a receiver at 115200 in about a second instead of about five.  If it is
  not there, the rest of the range is tried as normal (`-B` still changes the receiver to that rate afterwards).
* `-M, --minbaud` and `-X, --maxbaud` set the lowest and highest rate tried.  The defaults can be set in the
  `[gpsctl]` section of `gpsctl.conf` as `autobaud minimum` and `autobaud maximum`; the command line overrides the file.
  A minimum above the maximum is an error.  A `-B` rate outside the range is still tried first.

## Examples
To autodetect the current baud rate, set the pi and u-blox module to 115200 baud, configure stationary mode, Galileo satellites, and NMEA ZDA output only, and to see what it's doing

     gpsctl -a -B 115200 --configure_for_timing -vv

To autodetect the current baud rate and configure satellites only

     gpsctl -a --satellites

To view the current u-blox config

     gpsctl -a -Q config

Verbosity: with `-v` and `-vv` gpsctl says what it is doing (which baud rates it tried, what it changed). Failures
always print the reason, even with `-q`.

## Configuration file
The file `/etc/gpsctl.conf` (or the file named by `GPSCTL_CONF`) is read for configuration options. It is optional.

The `[gpsctl]` section determines which serial port gpsctl uses, the default verbosity, the default sync method and the range of baud rates that autobaud (`-a`) tries.
The other sections are used by `--galileo`, `--satellites` and `--configure_for_timing`, and override the defaults
if present. Example configurations are in `etc/`: `gpsctl.conf` (the one below), `gpsctl.conf.loose` (Galileo,
and should almost always get a fix), `gpsctl.conf.ublox` (the u-blox 8 defaults) and `gpsctl.conf.default`
(the settings found after `--reset`).

Numbers must be written as numbers and nothing else: a value that isn't entirely a number, or is outside the range
its field allows, is reported on stderr and the default is used. Anything else on the line (units, remarks) goes in a
comment, which starts with ';' or '#' and runs to the end of the line, e.g.
`Antenna cable delay = 56 ; nanoseconds`. (A value can therefore not contain ';' or '#'.)

Yes/no values are decided by the first character (`y`, `t`, `1` or `on` for yes; `n`, `f`, `0` or `off` for no);
anything else is reported and the default is used. The NMEA version must be one of 2.1, 2.3, 4.0, 4.1, 4.10 or
4.11 (or 21, 23, 40, 41, 410, 411); anything else is reported and 4.1 is used. Dynamic model must be 0 or 2 to 10,
fix mode 1 to 3, minimum elevation -90 to 90, and UTC standard one of 0, 3, 5, 6, 7, 8 or 9.

Either BeiDou or GLONASS can be configured, but not both at the same time as GPS or Galileo. If neither GPS nor
Galileo is enabled, you can enable both BeiDou and GLONASS. If you try with GPS or Galileo enabled, the `[gpsctl]`
option "prefer beidou to glonass if both enabled" (default: no) decides which one is used.

An example which configures for Galileo satellites:

    #
    # example gpsctl.conf which enables Galileo
    #   C/No threshold reduced to 20 from the original gpsctl's 40
    #   This gets a fix on 14 satellites in my home testing with an indoor GPS antenna 
    #
    [gpsctl]
    port = /dev/ttyAMA0
    # sync method: ASCII = 1, NMEA = 2, UBX = 3
    sync method = 3
    verbosity = 0
    # the range of baud rates that -a (autobaud) tries; -M and -X on the command line override these
    autobaud minimum = 9600
    autobaud maximum = 921600
    # Only one of Beidou and GLONASS can be used at the same time
    # so rather than error out trying to enable both
    # choose which one to use
    prefer beidou to glonass if both enabled = no

    [NMEA]
    # enabled = true / false
    enabled = true
    # version = 2.1 / 2.3 / 4.0 / 4.1 / 4.11
    version = 4.1
    GGA = off
    GLL = off
    GSA = off
    GSV = off
    RMC = on
    VTG = off
    GRS = off
    GST = off
    ZDA = on

    [GPS]
    enabled = yes
    minimum channels=8
    maximum channels=16

    [SBAS]
    enabled = no
    minimum channels=1
    maximum channels=3

    [Galileo]
    enabled = yes
    minimum channels=4
    maximum channels=8

    [Beidou]
    enabled = no
    minimum channels=8
    maximum channels=16

    [IMES]
    enabled = no
    minimum channels=0
    maximum channels=8

    [QZSS]
    enabled = no
    minimum channels=0
    maximum channels=3

    [GLONASS]
    enabled = yes
    minimum channels=8
    maximum channels=14

    [Navigation Engine]
    # Dynamic model: Portable = 0, Stationary = 2, Pedestrian = 3, Automotive = 4, Sea = 5, Air1G = 6, Air2G = 7, Air4G = 8, Watch = 9, Bike = 10
    Dynamic model =                2
    # Fix mode: 2D only = 1, 3D only = 2, auto 2D/3D = 3
    Fix mode =                     2
    Fixed altitude (2D) =          0.00 ; meters
    Fixed altitude variance (2D) = 0.0000 ; meters^2
    Minimum elevation =            20 ; degrees
    Position DoP mask =            10.0
    Time DoP mask =                10.0
    Position accuracy mask =       40 ; meters
    Time accuracy mask =           40 ; meters
    Static hold threshold =        0 ; cm/s
    Dynamic GNSS timeout =         60 ; seconds
    Threshold above C/No =         8 ; satellites
    C/No threshold =               20 ; dBHz
    Static hold max distance =     0 ; meters
    # UTC Standard: Auto = 0, USNO = 3, EU = 5, GLONASS = 6, BeiDou = 7, NPLI = 8, NICT = 9
    UTC standard =                 3

    [Time Pulse]
    # the nanoseconds / microseconds after the numbers are just reminders, they don't mean anything to the config parser
    Antenna cable delay = 56 ; nanoseconds
    RF group delay = 20 ; ns
    Unlocked pulse period = 1000000 ; microseconds
    Unlocked pulse length = 0
    Locked pulse period = 1000000 ; microseconds
    Locked pulse length = 500000 ; microseconds
    User configurable delay = 0

## Status and testing
See [docs/STATUS.md](docs/STATUS.md): 79 unit tests; a comparison against the real C program in 137 command-line
scenarios (identical output and identical frames sent to a fake receiver); the main options run on a real u-blox M8
(protocol 18) on a Raspberry Pi UART. Not yet run on real hardware: the cross-compile step of `build.sh`, the
listen-first autobaud pass, UTC standards 5/8/9. The [Changelog](Changelog) includes the C program's history.

## Layout

| path | what |
|---|---|
| `src/` | the library: UBX/NMEA messages, serial layer, synchronizers, commands, config, CLI (`cli.rs`, `queries.rs`) |
| `src/bin/gpsctl.rs` | the `gpsctl` program |
| `examples/nmea_tool.rs` | older diagnostic tool (`cargo run --example nmea_tool -- --help`) |
| `etc/` | example configuration files: gpsctl, chrony, and the systemd units and drop-ins |
| `c-golden/` | golden data produced by the real C code, and the C-versus-Rust comparison scripts |
| `tools/fake_m8.py` | a fake receiver on a pty, for trying the tool without hardware |
| `docs/STATUS.md` | status, verification and findings |

## How is gpsctl licensed?
*gpsctl* is licensed with the permissive MIT license. The original program is by Tom Dilatush:
> Created: October 18, 2017
> Author: Tom Dilatush <tom@dilatush.com>  
> Github:  
> License: MIT
>
> Copyright 2017 Tom Dilatush
>
> Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
> documentation files (the "Software"), to deal in the Software without restriction, including without limitation
> the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
> to permit persons to whom the Software is furnished to do so.
>
> The above copyright notice and this permission notice shall be included in all copies or substantial portions of
> the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
> THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE A
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
> TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
> SOFTWARE.

This port is also released under the MIT licence (see `LICENSE`).

## History
The original *gpsctl* was written by Tom Dilatush. Phil Randal forked it, adding the configuration file,
Galileo and timing support, and much more; this Rust port follows his C version.

## In Memoriam

Tom Dilatush (aka slightlyloony on here, slightlyloony1 on Twitter), sadly passed away on February 14th, 2024, from Amyotrophic lateral sclerosis (ALS), also known as motor neurone disease (MND).

If you find this program useful, please consider donating to your local ALS / MND charity.
