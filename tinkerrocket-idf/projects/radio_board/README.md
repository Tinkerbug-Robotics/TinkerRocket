# radio_board — LoRa daughterboard firmware (transparent UART modem)

Firmware for the V8 radio daughterboard (#409): an ESP32-S3 that owns the
LoRa radio (LLCC68) and exposes it to a host over UART as a **transparent
modem**. The same hardware + firmware binary serves both ends of the link —
plugged into the rocket (out computer, #410) or the base station (#414) —
and matched pairs are swapped as a unit (e.g. a future higher-power module).

**Transparent** means: `TX_FRAME`/`RX_FRAME` payloads are the exact bytes the
host would have handed to / read from `TR_LoRa_Comms` on a direct-SPI board.
The modem never parses them. The on-air format (`LORA_PROTO_VERSION`,
network ids, hop schedules) is owned entirely by the hosts, so deployed
direct-radio base stations interoperate with a V8 rocket unchanged.

## The board

Target is the as-built **LoRa Board V3** daughterboard: an ESP32-S3 (U28), an
EBYTE **E220-900MM22S** radio module (U14 — an LLCC68 with a fixed 900 MHz
matching network, its own 32 MHz crystal, and an RF switch), a W25Q64 boot
flash, USB-C for console+programming, and a 4-pin JST-SH host link. A later
board revision exists in `hardware/lora-daughterboard` (S3RH2, W25Q128); its
GPIO net map is pad-by-pad identical, so only the flash size in
`sdkconfig.defaults` is revision-specific.

Every pin in [main/config.h](main/config.h) is a schematic net verified
against the board's `.kicad_pcb` pad→net map:

| Signal | GPIO | Net | Goes to |
|--------|------|-----|---------|
| Host UART TX | 6 | `LoRa_TX` | J6.3 → host's RX |
| Host UART RX | 5 | `LoRa_RX` | J6.4 ← host's TX |
| SPI SCK / MISO / MOSI | 17 / 33 / 21 | `L_SCK` / `L_MISO` / `L_MOSI` | U14 15 / 12 / 13 |
| Radio CS / DIO1 / RST / BUSY | 18 / 2 / 38 / 34 | `L_CS` / `L_DI01` / `L_RST` / `L_BUSY` | U14 14 / 20 / 3 / 11 |
| RF switch RXEN | 35 | `L_RXEN` | U14 10 |
| RX LED (red D5) | 7 | `IND_2` | R60 → D5 |
| TX LED (blue D4) | 8 | `IND_1` | R59 → D4 |

Two things the schematic decides that the firmware must respect:

- **The TX half of the RF switch is not ours.** U14's DIO2 (19) is shorted to
  TXEN (9) on the board, which is the split EBYTE prescribes; only RXEN
  reaches a GPIO. `TR_LoRa_Comms` handles this with `setDio2AsRfSwitch(true)`
  plus `setRfSwitchPins(rxen, NC)`.
- **DIO3 is floating on purpose.** The E220 carries its own passive crystal,
  so this radio must never be set up for a DIO3-powered TCXO.

### Host connector (J6)

J6 mates 1:1 with rocket-computer J5 and base-station J6.

| Pin | Daughterboard net | Direction | Rocket host | Base-station host |
|-----|-------------------|-----------|-------------|-------------------|
| 1 | `VSS` | — | Q10-switched ground (`LoRa_ACT`) | hard GND |
| 2 | `+BATT` | in | 6.4–8.4 V pack | ~4.6 V `V_LORA` |
| 3 | `LoRa_TX` (GPIO6) | **out** | OC GPIO10 (its RX) | BS GPIO36 (its RX) |
| 4 | `LoRa_RX` (GPIO5) | **in** | OC GPIO11 (its TX) | BS GPIO35 (its TX) |

Every board names these nets from *its own* perspective, so the same two names
appear on both ends of a crossed pair — **the cable pin number is the only
unambiguous reference.** Deriving the direction from the net name is what put
`HOST_UART_TX = 5` in this file originally, which would have driven two
push-pull CMOS outputs onto one wire.

## Build & flash

```sh
. ~/esp/esp-idf-v6.0/export.sh      # IDF v6.0, same as FC/OC
idf.py build
idf.py -p <port> flash monitor
```

Console and programming are over USB-C (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`);
UART0 is unconnected on this board, and the host UART owns its own pins.

### Hardware constraints (enforced, not just documented)

- **No Wi-Fi, no BLE, ever.** The S3's `LNA_IN` is left floating with no
  matching network. `CONFIG_BT_ENABLED` fails the build. Wi-Fi has no
  equivalent switch — `CONFIG_ESP_WIFI_ENABLED` is a non-prompt SoC-capability
  symbol that is always `y` on an S3 — so that half is a rule stated in
  [main/main.cpp](main/main.cpp): nothing here may call `esp_wifi_init()`.
- **No PSRAM.** `CONFIG_SPIRAM` fails the build. On the as-built board there is
  none to enable; on the S3RH2 revision the in-package PSRAM shares VDD_SPI's
  internal ~14 Ω with the boot flash, and both active sags that rail under
  either part's 2.7 V minimum.

## UART link

- Framing: `tr_msg` codec ([TR_MsgCodec.h](../../components/TR_UART_Link/TR_MsgCodec.h)) —
  `AA 55 AA 55 | type | len | payload | CRC16be(type+len+payload)`, byte-identical
  to the FC↔OC I2C/I2S framing. UART is a byte stream, so a per-byte deframer
  hunts SOF and resynchronizes through garbage (drops are counted, never silent).
- 921600 baud 8N1, no RTS/CTS — flow control is in-band (below).
- Message set: [RadioModemProtocol.h](../../components/TR_UART_Link/RadioModemProtocol.h)
  (`PROTOCOL_VERSION` = 1). All codes for this link live in that one file.

### Messages

| Dir | Type | Code | Payload |
|-----|------|------|---------|
| H→M | `TX_FRAME` | 0x01 | `TxFrameHeader{seq}` + air bytes (≤ `MAX_AIR_FRAME` = 247) |
| H→M | `SET_CONFIG` | 0x02 | `RadioConfigData` (freq/SF/BW/CR/power/preamble/flags, `start_rx`) |
| H→M | `HOP_FREQ` | 0x03 | `HopFreqData` — per-packet hop; fire-and-forget, mid-TX refusal keeps channel |
| H→M | `START_RX` | 0x04 | — |
| H→M | `GET_STATUS` | 0x05 | — → `STATUS` |
| H→M | `GET_IDENTITY` | 0x06 | — → `IDENTITY` |
| H→M | `START_SCAN` | 0x07 | `ScanRequestData` → `SCAN_RESULT` when the sweep completes |
| M→H | `RX_FRAME` | 0x81 | `RxFrameHeader{rssi,snr}` + air bytes |
| M→H | `TX_RESULT` | 0x82 | `TxResultData{seq, ok}` — the credit return |
| M→H | `STATUS` | 0x83 | `ModemStatusData` (also sent as the ack to every `SET_CONFIG`) |
| M→H | `IDENTITY` | 0x84 | `ModemIdentityData` |
| M→H | `SCAN_RESULT` | 0x85 | `ScanResultHeader` + int8 RSSI samples |
| M→H | `BOOT` | 0x86 | `ModemIdentityData` — host must reset its credit window |

### Flow control (in-band credits)

The radio is orders of magnitude slower than the UART. The modem queues up to
`TX_QUEUE_CAPACITY` (8) frames; each accepted `TX_FRAME` seq is **always**
answered with a `TX_RESULT` — after airtime is spent, on failure, on
oversize/queue-full rejection, or when the TX watchdog clears a wedge. The
host keeps ≤ 8 unacknowledged seqs outstanding. `BOOT` resets the window
(modem rebooted, queue empty). No frame is ever silently dropped.

### Identity / capabilities (swappable modules)

`IDENTITY`/`BOOT` report protocol version, radio chip, band edges, max TX
power, and the firmware version (git sha, embedded via `esp_app_desc`).
Hosts clamp their radio config to the reported capabilities and must refuse
loudly on a protocol-version mismatch. The modem additionally clamps TX power
itself — a mismatched pair degrades, it never transmits out of spec.

The band edges reported are the **module's** (850–930 MHz for the
E220-900MM22S), not the bare LLCC68 die's 150–960 MHz. The die's range is not
reachable through a fixed 900 MHz matching network and RF switch, and
reporting capability a host could act on but the hardware cannot deliver
defeats the handshake. A higher-power or different-band module variant changes
these constants and nothing else.

### Modem behavior

- Boots listening on defaults (915 MHz / SF10 / BW125) so a bench modem is
  immediately visible; the host's `SET_CONFIG` replaces this. The host owns
  config — the modem has **no NVS**.
- Half-duplex policy matches `TR_LoRa_Comms`: auto-return to RX after every
  TX and read; TX is deferred while a spectrum scan is active.
- If the radio fails to init, the modem stays alive UART-side
  (`STATUS.radio_enabled = 0`, all TX_FRAMEs fail-fast) and the next
  `SET_CONFIG` retries a full `begin()`.

## Bench validation

The test host is [`tools/bench_radio_modem.py`](../../../tools/bench_radio_modem.py)
— a laptop-side speaker of this protocol, so none of the steps below need a
rocket or a base station wired up to get started.

**Wiring.** 3.3 V TTL adapter only — the S3 is not 5 V tolerant.

| J6 | Adapter |
|----|---------|
| 1 `VSS` | GND |
| 2 `+BATT` | 6.4–8.4 V bench supply, or leave it and power over USB-C |
| 3 | adapter **RX** |
| 4 | adapter **TX** |

**Step 0 — codec, no hardware.** Also runs in CI
(`tests/integration/test_radio_modem_codec.py`):

```bash
python3 tools/bench_radio_modem.py --selftest
```

**Step 1 — is the radio alive at all?** USB-C only; no UART adapter, no host,
no J6. The board in hand was subjected to a **backwards power connector**, so
this comes before anything about the link — there is no point debugging a
protocol against a dead module.

```bash
idf.py -p <usb-port> flash monitor
```

The firmware probes the E220 at the pin level before the driver touches the
SPI bus, because a failed `begin()` cannot tell a dead module from a rejected
config. BUSY and DIO1 are push-pull module outputs with no board pull
resistors, so the probe fights them with the S3's internal pulls: a level that
follows the pull means nothing is driving the pin.

| Boot line | Reading |
|-----------|---------|
| `radio probe: module ALIVE — BUSY released N us after reset and is driven` | The module has power, booted, and drives its pins. If `begin()` still fails after this, the fault is on SPI or in config — not the module. |
| `radio probe: BUSY is FLOATING` | Nothing drives U14 pad 11. Module unpowered, dead, or open joints. **Measure +3V3 at U14 pad 1 first.** This is the expected signature of reverse-polarity damage. |
| `radio probe: BUSY driven but STUCK HIGH past 50 ms` | Powered and driving, but never finished its internal boot — damaged die, or a bad NRST/SPI joint. |
| `radio init FAILED (RadioLib <code>)` | The RadioLib code narrows it further: `-2` is chip-not-found (SPI silent), the `-70x` family are SPI command failures. |
| `radio up, listening at 915.0 MHz SF10` | Radio is good; move on. |

Two LED blinks at boot say the S3 itself is running — the only such sign if
the board is powered from J6 with no USB attached. (D6 is a hardwired power
LED and proves only that the rail is up. Note R59/R60 are 10 kΩ, so the blue
LED may be very dim or dark; see the config.h bench note.)

If the S3 does not enumerate over USB at all, the reverse-polarity damage
reached past the radio and the rest of this plan is moot.

**Step 2 — the link is alive.** Expect `BOOT` within ~1 s of power-up and an
`IDENTITY` on demand. `protocol=v1`, `chip=LLCC68`, `band=850.000-930.000`,
and an `fw=` git sha that matches what you flashed:

```bash
python3 tools/bench_radio_modem.py -p /dev/tty.usbserial-XXXX --identity --status
```

No reply is almost always wiring: TX/RX swapped at the adapter, or a baud
mismatch. `STATUS` also reports `uart_crc_fail` / `uart_resync` — nonzero
there means the link is up but noisy, which is a different problem from
silent.

**Step 3 — the radio came up.** `SET_CONFIG` is acked with a `STATUS`, and
that ack is the only proof the LLCC68 actually initialised — `radio=DOWN`
means the modem is alive with dead RF:

```bash
python3 tools/bench_radio_modem.py -p PORT --config 915 125 10 7 20
```

**Step 4 — flow control.** Push more frames than the 8-deep credit window and
confirm `0 unanswered`. That is the "a TX frame must never be silently
dropped" clause of #409 as a number:

```bash
python3 tools/bench_radio_modem.py -p PORT --tx DEADBEEF --repeat 50
```

**Step 5 — over the air (the #409 acceptance criterion).** With a second
radio on the same parameters, `--listen` on one end and `--tx` on the other;
`RX_FRAME` carries the air bytes back verbatim with RSSI/SNR. Then repeat
against an **unmodified deployed base station**, transmitting bytes its
`LORA_PROTO_VERSION` parser accepts — `--tx` of arbitrary hex proves the
tunnel and the credit return, not application interop.

`--scan 902 928 500 30` sweeps the band and prints the peak, which is the
quickest way to confirm the RF switch and antenna path are doing something.

**Not yet run on hardware.** Everything above is bench-ready but unexecuted:
the daughterboard has not been brought up. Steps 1–5 need a board on the bench
and a go-ahead.

## Tests

- `tests_cpp/test_uart_link_codec.cpp` — golden wire-format bytes (literal, so
  a CRC-parameter change is caught), deframer resync/corruption behaviour,
  wire-stable struct layouts.
- `tests/integration/test_radio_modem_codec.py` — pins the bench host's
  independent Python codec to the same golden frames, so the two
  implementations of this wire format cannot drift apart.
