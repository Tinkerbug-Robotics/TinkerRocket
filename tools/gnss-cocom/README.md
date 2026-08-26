# PX1105R COCOM bench rig (#491)

Measures how the SkyTraq PX1105R/PX1125R enforce the COCOM limits: as an **AND**
(block only when altitude *and* velocity are exceeded together) or
**independently**, and whether the altitude gate is 18 km or 80 km. A rocket
trajectory almost never trips both at once — boost is fast-but-low, a space-shot
apogee is high-but-slow — so if the gate is a true AND these receivers
essentially never blank in flight, which is the whole reason to look at SkyTraq
for high/fast flights.

Scope caveat, restated from the issue because it is easy to lose: **this rig
tests the firmware gate only.** The injected signal is clean and vibration-free.
It answers "does the gate block?", not "is boost-phase data usable" — that is
the separate high-g/transonic dynamics problem (#174 / #249 / #262).

## Contents

| Path | What it is |
|---|---|
| `firmware/gnss_passthrough/` | Byte-transparent USB ↔ receiver UART bridge. Builds for RP2040, ESP32-S3 and ESP32-C3 |
| `gnss_nmea_monitor.py` | Host monitor: timestamps, checksums, and the FIX / BLOCKED / NO_LOCK classifier |
| `skytraq_cmd.py` | Sends SkyTraq binary configuration frames through the bridge |
| `skytraq_binary.py` | SkyTraq binary message parsing (AN0039): `0xDF` nav state, `0xE7` per-SV C/N0 |

Still to be written, once the RF side exists: the trajectory generator (the
pyproj snippet in #491) and the capture-vs-trajectory correlation step.

## Which MCU the receiver is actually on

This matters more than it sounds. The TinkerNav family splits into two shapes:

- **Dual-MCU boards** (V7–V17 and the "V20 - ESP32" BOM): **RP2040 + ESP32-C3 +
  PX1125R**, with *two* USB-C ports. The receiver hangs off the **RP2040**;
  the C3 is a companion radio. A passthrough flashed to the C3 finds nothing on
  every GPIO, because there is nothing there to find.
- **Single-MCU boards** (V25): **ESP32-S3 + PX1105R**, one USB-C.

On a dual-MCU board, plugging into the "wrong" USB port gets you a working,
enumerating MCU that simply cannot see the GNSS. Both ports appear as
`/dev/cu.usbmodemNNNN`; tell them apart by USB vendor — Raspberry Pi `2E8A` is
the RP2040 side, Espressif `303A` the ESP32 side.

RP2040 pin map, from Tinkerbug's own rover firmware
(`TinkerRTKLoRa/RP2040_Rover_LoRaRadio`) rather than a schematic:

| Signal | RP2040 | Note |
|---|---|---|
| receiver TXD → MCU | **GPIO1** | `Serial1` default RX |
| MCU → receiver RXD | **GPIO0** | `Serial1` default TX |
| RTCM → receiver RXD2 | GPIO4/5 | `Serial2`, unused here |

A caution about the design files: the folder `TinkerNav V20 - ESP32/` contains a
`.kicad_sch` and `.kicad_pcb` describing a **single-MCU ESP32-S3** board, while
the `.csv` BOM beside them lists the RP2040 + ESP32-C3 build. The schematic has
been overwritten at some point and does not describe the board the BOM does.
Trust the BOM, the firmware, and the silicon over that schematic.

## Why a custom sketch

The stock TinkerNav rover/base firmware consumes the GNSS stream into
`TR_SkyTraqNMEA`'s parser and serves a decoded subset over a web UI:

```cpp
while (Serial1.available()) { char c = Serial1.read(); sky.feedChar(c); }
```

Raw sentences never reach USB. This test needs the opposite — every byte,
host-timestamped, including the GSV/GSA traffic that separates a COCOM block
from a loss of lock — plus the reverse direction intact so SkyTraq binary
configuration frames can be pushed from the host.

## Board facts

Extracted mechanically from `TinkerNav V25.kicad_sch` in the TinkerNav repo
(`kicad-cli sch export netlist`), not transcribed:

| Net | ESP32-S3 (U3) | PX1105R (U2) | Role |
|---|---|---|---|
| `GPS_RX` | IO47 | pin 3 `RXD` | ESP32 → receiver |
| `GPS_TX` | IO48 | pin 2 `TXD` | receiver → ESP32 |
| `GPS_RXD2` | IO16 | pin 15 `RXD2` | RTCM in, unused here |
| `NEOPIX` | IO26 | — | status LED |

U3 is an **ESP32-S3-MINI-1-N8**, U2 a **PX1105R**, J7 a USB-C wired straight to
the ESP32's D+/D− — native USB, no bridge chip, so the port only exists while
firmware is running.

Watch the naming trap: TinkerNav's `config.h` names IO47 `GNSS_TX` and IO48
`GNSS_RX` from the *ESP32's* point of view, then passes them as
`Serial1.begin(baud, cfg, rxPin, txPin)` → `(GNSS_RX, GNSS_TX)`. The sketch here
names its pins for the ESP32 UART role instead.

### RF path — check before connecting an SDR

`J2` is an **edge-mount SMA female** (`RFPC-SMA28-F`) straight to the receiver's
antenna port, so injection needs no U.FL pigtail — one line item off the #491
shopping list.

In the V25 schematic the RF net has exactly two nodes, `J2.1` and `U2.11
RF_IN`, with no bias tee, feed inductor, or series cap, and `U2.14 VCC_RF` is
explicitly unconnected. On that evidence the SMA centre pin carries **no DC
bias**, which would make the DC block in #491's parts list unnecessary *for this
board* and the "antenna-open detection" gotcha moot.

**Verify with a DMM on the actual board before trusting that.** It is a
ten-second measurement protecting a $150–300 SDR, the analysis above is of the
V25 design rather than the board in your hand, and other GNSS boards here — the
repo's own `gnss-px1105r-18mm-highpower-ext-ant`, by its name — may well bias
their antenna port. Keep the DC block inline if there is any doubt; it costs a
fraction of a dB.

**Conducted only.** GPS L1 is a live, protected band: cable, attenuators and a
shielded path, never a radiating antenna.

## Building and flashing

The pin map is **not portable between ESP32 variants** — the V25's IO47/IO48 do
not exist on an ESP32-C3, which stops at GPIO21 — so the sketch carries per-chip
defaults and prints the pins it actually compiled in at boot.

ESP32-S3 (TinkerNav V25, 319 KB / 24% of flash):

```bash
arduino-cli compile --fqbn 'esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=8M,PSRAM=disabled' tools/gnss-cocom/firmware/gnss_passthrough
```

ESP32-C3 (307 KB / 23%):

```bash
arduino-cli compile --fqbn 'esp32:esp32:esp32c3:CDCOnBoot=cdc' tools/gnss-cocom/firmware/gnss_passthrough
```

`CDCOnBoot=cdc` is not optional — without it `Serial` is not the USB port and
nothing reaches the host. Override the pins for a hand-wired board:

```bash
arduino-cli compile --fqbn 'esp32:esp32:esp32c3:CDCOnBoot=cdc' --build-property "compiler.cpp.extra_flags=-DGNSS_UART_RX_PIN=4 -DGNSS_UART_TX_PIN=5" tools/gnss-cocom/firmware/gnss_passthrough
```

Flash with the board's port substituted:

```bash
arduino-cli upload -p /dev/cu.usbmodemXXXX --fqbn 'esp32:esp32:esp32c3:CDCOnBoot=cdc' tools/gnss-cocom/firmware/gnss_passthrough
```

### What it does at boot

1. **Baud autodetect** on the configured pin, scoring valid NMEA checksums at
   each candidate rate. A receiver left reconfigured by earlier bench work
   otherwise presents as line noise and reads like a dead board.
2. **Pin scan**, if nothing parsed. It binds UART1's RX to each candidate GPIO
   in turn and scores checksums, so a board whose schematic is not to hand
   reports its own wiring. Pins that carry traffic but no valid NMEA get a
   second pass across the other baud rates, because "wrong rate" and "nothing
   wired here" are different faults.
3. **Bridge** on whatever was found.

Autobaud scores *NMEA* checksums, so on a binary-only receiver it finds no lock
and settles on 115200 — which is correct, and the bridge then passes the binary
through fine. Re-locking is gated on having had a lock to lose, so a receiver
that never emits NMEA gets a quiet bridge rather than a re-probe every few
seconds. That gate matters: re-probing tears the UART down and back up, which
eats the ACKs `skytraq_cmd.py` is waiting on — the bridge would otherwise fight
the tool trying to configure the receiver.

### The firmware will not drive a pin it is guessing about

Reading a pin is always safe. *Driving* one is the only thing here that can
damage hardware — if the pin is another device's output, the two drivers fight.
So the sketch tracks whether its TX pin is **known** or **guessed**, and only
ever transmits on a known one:

| Build | TX | Why |
|---|---|---|
| ESP32-S3 default | driven (IO47) | the V25 map came from the netlist |
| explicit `-DGNSS_UART_TX_PIN=n` | driven | the builder asserted it |
| ESP32-C3 default | **unbound** | the pin pair is a guess |
| after the scan relocates RX | **unbound** | the map it came from was wrong |

An unbound TX gives a listen-only bridge: NMEA still flows to the host, which is
all #491's measurement needs. Sending SkyTraq configuration frames needs the real
transmit pin, and that should come from a schematic rather than a coin flip. All
probing and scanning is listen-only regardless of the above.

The boot log states which case applies, so a listen-only bridge is never a
silent surprise.

It re-locks after a silence window, so a configuration frame that changes the
receiver's baud mid-session is recovered from. A COCOM block does not trigger
that path: a withheld fix still emits well-formed sentences with empty fields.

Status LED (where the board has one): green = valid NMEA flowing, amber = bytes
arriving but nothing parses, red = silence.

### Before overwriting a board

Back the flash up first — it makes the whole thing reversible:

```bash
esptool --port /dev/cu.usbmodemXXXX read-flash 0x0 0x400000 backup.bin
```

Restore with `write-flash 0x0 backup.bin`. `esptool` ships inside the Arduino
core at `~/Library/Arduino15/packages/esp32/tools/esptool_py/5.1.0/esptool`.

## Running the monitor

```bash
python3 tools/gnss-cocom/gnss_nmea_monitor.py --list
```

```bash
python3 tools/gnss-cocom/gnss_nmea_monitor.py --seconds 30
```

Capture a run for later analysis, which is what the real tests want:

```bash
python3 tools/gnss-cocom/gnss_nmea_monitor.py --log t1_velramp.nmea
```

```bash
python3 tools/gnss-cocom/gnss_nmea_monitor.py --replay t1_velramp.nmea
```

## The receiver speaks binary, and that is fine

The monitor reads **both** NMEA and SkyTraq binary, auto-detecting per frame, so
what the receiver happens to be emitting does not matter. Tell them apart by the
first bytes:

| First bytes | What it is |
|---|---|
| `$G...` | NMEA |
| `A0 A1 ..` | SkyTraq binary — `0xDF` navigation state, `0xE7` per-SV C/N0 |
| `D3 00 ..` | RTCM3 — the receiver is an RTK **base**, streaming corrections and reporting no position |

**Binary is the better input for this measurement**, not a fallback:

- `0xDF` carries an explicit **Navigation State** enum, so "no fix" is a stated
  value rather than something inferred from empty NMEA fields.
- `0xDF` carries ECEF **velocity**, so speed comes from the receiver instead of
  being reconstructed from position differences.
- `0xE7` carries per-satellite C/N0 **and** a "used in fix" bit — exactly the
  tracking-versus-position split the COCOM verdict turns on.

Layouts are in `skytraq_binary.py`, taken from SkyTraq Binary Protocol AN0039
v1.4.42 and cross-checked against bytes captured from the bench receiver.

### Reconfiguring the receiver

```bash
python3 tools/gnss-cocom/skytraq_cmd.py --nmea
```

That runs the sequence: leave base mode, stop RTCM, switch output to NMEA,
enable GGA/RMC/GSA/GSV. It defaults to **SRAM only**, so a power cycle puts the
receiver back as it was; `--flash` persists.

**A caution learned the hard way.** On the PX1125R firmware on this bench board
(`SKYTRAQ03.00.01,01.07.33`), the NMEA switch does not work and cannot be made
to: `0x09` output-type ACKs and changes nothing, `0x1E` and `0x6A/0x01` NACK,
and a restore-defaults plus a physical power cycle leaves it emitting binary.
Config changes *do* apply — `--rtcm off` demonstrably stops the RTCM stream and
survives a power cycle — so this is the firmware ignoring one specific setting,
not a broken link. **Do not burn time fighting it; read the binary.**

`--factory-reset` restores defaults and cold-restarts. It **wipes the stored
configuration, including any base survey position**, is not undone by a power
cycle, and so requires `--yes-wipe-config` on top. It does *not* bring back NMEA
on this firmware.

Note also that ACKs must be matched against the command id. The receiver streams
continuously, so an ACK for an earlier command is nearly always sitting in the
buffer, and accepting any ACK reports success for commands that were ignored.

### What it decides

Each epoch is classified as one of three states, and the distinction *is* the
measurement:

- **FIX** — position reported.
- **BLOCKED** — satellites still tracked at healthy C/N0, position withheld.
  GGA quality 0 and RMC status `V` while GSV still lists strong satellites. A
  sharp, reversible transition at the limit. **This is the COCOM signature.**
- **NO_LOCK** — C/N0 collapsed and satellites dropped out first.

Reading position alone cannot tell BLOCKED from NO_LOCK, so tracking is parsed
alongside and the verdict comes from both — GSV/GSA on NMEA, `0xE7` on binary.

Each transition also records the **last good fix** before it. A COCOM block
withholds velocity along with position, so the state *at* the transition reads
0 m/s; the speed that matters is the one from the sample just before it blanked. The thresholds are
`TRACKED_CN0_DBHZ = 30.0` over `TRACKED_MIN_SATS = 4` — comfortably above where
a receiver starts dropping satellites and well below the 40–45 dBHz the
injection is tuned for, so the classifier is not sensitive to the exact level.

The summary prints time-by-verdict and a transition log. Correlating a
transition timestamp against the injected trajectory's altitude and velocity at
that instant is what yields the threshold.

## Test matrix

Ramps isolate one variable at a time (full trajectory generator in #491):

| Test | Trajectory | Only thing exceeded | If AND | If independent |
|---|---|---|---|---|
| 0 baseline | static / 50 m/s @ 1 km | nothing | FIX | FIX |
| 1 velocity ramp | hold 5 km, 0→900 m/s over 90 s | velocity | FIX throughout | BLOCKED at ~515 m/s |
| 2 altitude ramp | hold 50 m/s, 10→85 km over 120 s | altitude | FIX throughout | BLOCKED at 18 or 80 km |
| 3 combined | 25 km while 0→700 m/s over 60 s | both | BLOCKED when both | already blocked in 1 |

Test 1 settles AND-vs-OR; test 2 reveals the altitude gate and its threshold;
test 3 confirms.

## Injection level

Target roughly −100 dBm at `RF_IN` — real GPS is about −130 dBm plus the ~30 dB
an active antenna's LNA would have added. Start with **more** attenuation
(70–80 dB) and low HackRF gain and work down: too hot saturates the front end
and mimics "no signal", which is easy to misread as a failed rig. Reduce until
C/N0 lands at 40–45 dBHz and it fixes. There is ±10–15 dB of AGC tolerance.

## Still needed

The SDR half of #491 is unbuilt: HackRF One, attenuator kit, SMA jumpers, and
`gps-sdr-sim` with a current RINEX ephemeris. The pieces here are the parts that
work without any of that.
