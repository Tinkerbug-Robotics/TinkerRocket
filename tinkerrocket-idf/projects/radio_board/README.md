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

## Build & flash

```sh
. ~/esp/esp-idf-v6.0/export.sh      # IDF v6.0, same as FC/OC
idf.py build
idf.py -p <port> flash monitor
```

LoRa, RXEN, and indicator-LED pins in [main/config.h](main/config.h) are the
V8 daughterboard schematic values; the **host-link UART pins are still
placeholders** (TODO in config.h). The TX LED lights for the duration of each
transmission; the RX LED pulses per received air packet.

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

1. **UART loopback** (no radio): drive with a USB-UART, check `BOOT`,
   `IDENTITY`, `STATUS`, and TX_RESULT fail-fasts.
2. **Over-the-air**: devkit + LLCC68 breakout driven over UART, exchanging
   frames with an **unmodified existing base station** — proves the
   transparent tunnel end-to-end (acceptance criterion of #409).

## Tests

Host-side codec/protocol tests: `tests_cpp/test_uart_link_codec.cpp`
(golden wire-format bytes, deframer resync/corruption behavior, wire-stable
struct layouts).
